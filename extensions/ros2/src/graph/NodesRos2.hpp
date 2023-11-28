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
#include <visualization_msgs/msg/marker_array.hpp>
#include <Ros2InitGuard.hpp>

struct Ros2PublishPointsNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<Ros2PublishPointsNode>;

	void setParameters(const char* topicName, const char* frameId,
	                   rgl_qos_policy_reliability_t qosReliability = QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_durability_t qosDurability = QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_history_t qosHistory = QOS_POLICY_HISTORY_SYSTEM_DEFAULT, int32_t qosHistoryDepth = 10);

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	~Ros2PublishPointsNode() override = default;

private:
	DeviceAsyncArray<char>::Ptr inputFmtData = DeviceAsyncArray<char>::create(arrayMgr);

	std::shared_ptr<Ros2InitGuard> ros2InitGuard;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros2Publisher;
	sensor_msgs::msg::PointCloud2 ros2Message;

	void updateRos2Message(const std::vector<rgl_field_t>& fields, bool isDense);
};


struct Ros2PublishPointVelocityMarkersNode : IPointsNodeSingleInput
{
	using Ptr = std::shared_ptr<Ros2PublishPointVelocityMarkersNode>;

	void setParameters(const char* topicName);
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32};
	}

	// Node
	void validateImpl() override;
	void enqueueExecImpl() override;

	~Ros2PublishPointVelocityMarkersNode() override = default;

private:
	std::shared_ptr<Ros2InitGuard> ros2InitGuard;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr linesPublisher;
	HostPinnedArray<Vec3f>::Ptr pos = HostPinnedArray<Vec3f>::create();
	HostPinnedArray<Vec3f>::Ptr vel = HostPinnedArray<Vec3f>::create();

	visualization_msgs::msg::Marker makeLinesMarker();
};