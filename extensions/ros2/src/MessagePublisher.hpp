// Copyright 2023 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * Abstract base class defining the interface for ROS2 publishing strategies.
 *
 * This class provides a unified interface for different publishing approaches in ROS2,
 * such as standard publishing and zero-copy loaned message publishing.
 *
 * @tparam MessageT The ROS2 message type to be published
 */
template<typename MessageT>
struct MessagePublisher
{
	/**
	 * This method provides access to the message object, allowing
	 * the caller to modify its content before publishing.
	 * The implementation may create a new message, return an existing one,
	 * or borrow a loaned message from the middleware.
	 */
	virtual MessageT& getMessage() = 0;

	/**
	 * Publishes the current message (as provided by getMessage()) to subscribers.
	 */
	virtual void publish() = 0;

	/**
	 * Returns the topic name this publisher is configured to publish to.
	 */
	virtual std::string getTopicName() const = 0;

	/**
	 * Returns the Quality of Service (QoS) settings for this publisher.
	 */
	virtual rclcpp::QoS getQos() const = 0;

	virtual ~MessagePublisher() = default;
};

/**
 * Implementation of MessagePublisher using standard ROS2 publishing mechanism.
 *
 * This class provides a traditional ROS2 publishing approach where a message object
 * is maintained internally, modified by the user, and then copied to the middleware
 * during publishing. This is the default and most compatible publishing strategy.
 *
 * @tparam MessageT The ROS2 message type to be published
 */
template<typename MessageT>
struct Ros2MessagePublisher : public MessagePublisher<MessageT>
{
	explicit Ros2MessagePublisher(rclcpp::Node& node, std::string topicName, rclcpp::QoS qos)
	{
		publisher = node.create_publisher<MessageT>(topicName, qos);
	}

	MessageT& getMessage() override { return msg; };
	void publish() override { publisher->publish(msg); };
	std::string getTopicName() const override { return publisher->get_topic_name(); }
	rclcpp::QoS getQos() const override { return publisher->get_actual_qos(); }

private:
	MessageT msg{};
	rclcpp::Publisher<MessageT>::SharedPtr publisher;
};

#if RGL_BUILD_AGNOCAST_EXTENSION
#include <agnocast/agnocast.hpp>

/**
 * Implementation of MessagePublisher for ROS2 publishing using Agnocast middleware.
 *
 * This class uses Agnocast's API for zero-copy publishing using shared memory.
 * Messages are borrowed from Agnocast using a loan-based system that provides
 * an ipc_shared_ptr, which manages the lifetime of message objects transferred
 * between processes. The borrowed message is cached until published, at which
 * point it is moved to the publisher and the cache is reset.
 *
 * @tparam MessageT The ROS2 message type to be published
 */
template<typename MessageT>
struct AgnocastMessagePublisher : public MessagePublisher<MessageT>
{
	explicit AgnocastMessagePublisher(rclcpp::Node& node, std::string topicName, rclcpp::QoS qos)
	  : topicName(topicName), qos(qos)
	{
		publisher = agnocast::create_publisher<MessageT>(&node, topicName, qos);
	}

	MessageT& getMessage() override
	{
		if (!msgLoaned.has_value()) {
			msgLoaned.emplace(publisher->borrow_loaned_message());
		}
		return *(msgLoaned.value().get());
	};

	void publish() override
	{
		if (!msgLoaned.has_value()) {
			return;
		}
		publisher->publish(std::move(msgLoaned.value()));

		msgLoaned.reset();
	};

	std::string getTopicName() const override { return topicName; }
	rclcpp::QoS getQos() const override { return qos; }

private:
	std::optional<agnocast::ipc_shared_ptr<MessageT>> msgLoaned{};
	agnocast::Publisher<MessageT>::SharedPtr publisher{};

	std::string topicName;
	rclcpp::QoS qos;
};
#endif
