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

#include <RGLExceptions.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * Wrapper around RGL-specific ROS2 resources shared between RGL Nodes.
 * - Handles (de)initialization of rclcpp and creation of ROS2 node
 * - Keeps track of ROS2 publishers to avoid creating duplicates
 */
struct Ros2InitGuard
{
	rclcpp::Node& getNode() const { return *node; }

	static inline std::shared_ptr<Ros2InitGuard> acquire()
	{
		static std::weak_ptr<Ros2InitGuard> weak{};
		std::shared_ptr<Ros2InitGuard> shared{};
		if (auto locked = weak.lock()) {
			shared = locked;
		} else {
			shared = std::shared_ptr<Ros2InitGuard>(new Ros2InitGuard());
			weak = shared;
		}
		return shared;
	}

	template<typename T>
	rclcpp::Publisher<T>::SharedPtr createUniquePublisher(const std::string& topicName, const rclcpp::QoS& qos)
	{
		if (hasTopic(topicName)) {
			auto msg = fmt::format("ROS2 publisher with the same topic name ({}) already exist!", topicName);
			throw InvalidAPIArgument(msg);
		}
		auto publisher = node->create_publisher<T>(topicName, qos);
		publishers.insert({topicName, publisher});
		return publisher;
	}

	~Ros2InitGuard()
	{
		if (isRclcppInitializedByRGL) {
			rclcpp::shutdown();
			isRclcppInitializedByRGL = false;
		}
	}

private:
	Ros2InitGuard()
	{
		// Check if rclcpp initialized
		if (!rclcpp::ok()) {
			rclcpp::init(0, nullptr);
			isRclcppInitializedByRGL = true;
		}
		node = std::make_shared<rclcpp::Node>(nodeName);
	}

	bool hasTopic(const std::string& query)
	{
		for (auto it = publishers.begin(); it != publishers.end();) {
			it = it->second.expired() ? publishers.erase(it) : ++it;
		}
		return publishers.contains(query);
	}

private:
	rclcpp::Node::SharedPtr node;
	bool isRclcppInitializedByRGL{false};
	std::map<std::string, std::weak_ptr<rclcpp::PublisherBase>> publishers;
	inline static std::string nodeName = "RobotecGPULidar";
};
