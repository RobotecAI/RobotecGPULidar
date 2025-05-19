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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <radar_msgs/msg/radar_scan.hpp>

#include <graph/Node.hpp>
#include <graph/NodesCore.hpp>
#include <graph/Interfaces.hpp>

#include <Ros2InitGuard.hpp>
#include <MessagePublisher.hpp>

struct Ros2Node : IPointsNodeSingleInput
{
	Ros2Node() { ros2InitGuard = Ros2InitGuard::acquire(); }

	void enqueueExecImpl() final
	{
		if (!rclcpp::ok()) {
			throw InvalidPipeline("Unable to execute Ros2Node because ROS2 has been shut down.");
		}
		ros2EnqueueExecImpl();
	}

	void validateImpl() final
	{
		IPointsNodeSingleInput::validateImpl();
		ros2ValidateImpl();
	}

	virtual ~Ros2Node() = default;

protected:
	std::shared_ptr<Ros2InitGuard> ros2InitGuard;

	virtual void ros2EnqueueExecImpl() = 0;
	virtual void ros2ValidateImpl() = 0;
};

struct Ros2PublishPointsNode : Ros2Node
{
	using Ptr = std::shared_ptr<Ros2PublishPointsNode>;

	void setParameters(const char* topicName, const char* messageFrameId,
	                   rgl_qos_policy_reliability_t qosReliability = QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_durability_t qosDurability = QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_history_t qosHistory = QOS_POLICY_HISTORY_SYSTEM_DEFAULT, int32_t qosHistoryDepth = 10);

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

	~Ros2PublishPointsNode() override = default;

private:
	using MessageT = sensor_msgs::msg::PointCloud2;

	static void updateRos2MessageFields(sensor_msgs::msg::PointCloud2& ros2Message, const std::vector<rgl_field_t>& fields);

	DeviceAsyncArray<char>::Ptr inputFmtData = DeviceAsyncArray<char>::create(arrayMgr);

	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;
	std::string frameId{};
};


struct Ros2PublishPointVelocityMarkersNode : Ros2Node
{
	using Ptr = std::shared_ptr<Ros2PublishPointVelocityMarkersNode>;

	void setParameters(const char* topicName, const char* frameId, rgl_field_t velocityField);
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_VEC3_F32, velocityField}; }

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

	~Ros2PublishPointVelocityMarkersNode() override = default;

private:
	using MessageT = visualization_msgs::msg::Marker;

	std::string frameId;
	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;

	HostPinnedArray<Vec3f>::Ptr pos = HostPinnedArray<Vec3f>::create();
	HostPinnedArray<Vec3f>::Ptr vel = HostPinnedArray<Vec3f>::create();
	rgl_field_t velocityField;
};

struct Ros2PublishRadarScanNode : Ros2Node
{
	void setParameters(const char* topicName, const char* messageFrameId, rgl_qos_policy_reliability_t qosReliability,
	                   rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory, int32_t qosHistoryDepth);
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32, /* placeholder for amplitude */ PADDING_32};
	}

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

private:
	using MessageT = radar_msgs::msg::RadarScan;

	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;
	std::string frameId{};

	DeviceAsyncArray<char>::Ptr formattedData = DeviceAsyncArray<char>::create(arrayMgr);
	GPUFieldDescBuilder fieldDescBuilder;
};
