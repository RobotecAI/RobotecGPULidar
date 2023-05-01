// Copyright 2022 Robotec.AI
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

#include <rgl/api/extensions/ros2.h>

#include <graph/Node.hpp>
#include <graph/NodesCore.hpp>
#include <graph/Interfaces.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct Ros2PublishPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<Ros2PublishPointsNode>;

	void setParameters(
		const char* topicName, const char* frameId,
		rgl_qos_policy_reliability_t qosReliability = QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
		rgl_qos_policy_durability_t qosDurability = QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
		rgl_qos_policy_history_t qosHistory = QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
		int32_t qosHistoryDepth = 10);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	~Ros2PublishPointsNode();

private:
	VArray::Ptr inputFmtData = VArray::create<char>();

	std::string topicName{};
	std::string frameId{};

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros2Publisher;
	sensor_msgs::msg::PointCloud2 ros2Message;

	static rclcpp::Node::SharedPtr ros2Node;
	static std::string ros2NodeName;
	static std::set<std::string> ros2TopicNames;

	void updateRos2Message(const std::vector<rgl_field_t>& fields, bool isDense);
};
