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
#include <radar_msgs/msg/radar_scan.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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

	void setParameters(const char* topicName, const char* frameId,
	                   rgl_qos_policy_reliability_t qosReliability = QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_durability_t qosDurability = QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_history_t qosHistory = QOS_POLICY_HISTORY_SYSTEM_DEFAULT, int32_t qosHistoryDepth = 10);

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

	~Ros2PublishPointsNode() override = default;

private:
	DeviceAsyncArray<char>::Ptr inputFmtData = DeviceAsyncArray<char>::create(arrayMgr);

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros2Publisher;
	sensor_msgs::msg::PointCloud2 ros2Message;

	void updateRos2Message(const std::vector<rgl_field_t>& fields, bool isDense);
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
	std::string frameId;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr linesPublisher;
	HostPinnedArray<Vec3f>::Ptr pos = HostPinnedArray<Vec3f>::create();
	HostPinnedArray<Vec3f>::Ptr vel = HostPinnedArray<Vec3f>::create();
	visualization_msgs::msg::Marker marker;
	rgl_field_t velocityField;

	const visualization_msgs::msg::Marker& makeLinesMarker();
};

struct Ros2PublishRadarScanNode : Ros2Node
{
	void setParameters(const char* topicName, const char* frameId, rgl_qos_policy_reliability_t qosReliability,
	                   rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory, int32_t qosHistoryDepth);
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32, /* placeholder for amplitude */ PADDING_32};
	}

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

private:
	radar_msgs::msg::RadarScan ros2Message;
	rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr ros2Publisher;
	DeviceAsyncArray<char>::Ptr formattedData = DeviceAsyncArray<char>::create(arrayMgr);
	GPUFieldDescBuilder fieldDescBuilder;
};

struct Ros2PublishLaserScanNode : Ros2Node
{
	void setParameters(const char* topicName, const char* frameId,
	                   rgl_qos_policy_reliability_t qosReliability = QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_durability_t qosDurability = QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_history_t qosHistory = QOS_POLICY_HISTORY_SYSTEM_DEFAULT, int32_t qosHistoryDepth = 10);

	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {AZIMUTH_F32, DISTANCE_F32, TIME_STAMP_F64, INTENSITY_F32};
	}

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

private:
	DeviceAsyncArray<char>::Ptr inputFmtData = DeviceAsyncArray<char>::create(arrayMgr);

	sensor_msgs::msg::LaserScan ros2Message;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr ros2Publisher;

	HostPinnedArray<Field<AZIMUTH_F32>::type>::Ptr angles = HostPinnedArray<Field<AZIMUTH_F32>::type>::create();
	HostPinnedArray<Field<TIME_STAMP_F64>::type>::Ptr times = HostPinnedArray<Field<TIME_STAMP_F64>::type>::create();
	HostPinnedArray<Field<DISTANCE_F32>::type>::Ptr ranges = HostPinnedArray<Field<DISTANCE_F32>::type>::create();
	HostPinnedArray<Field<INTENSITY_F32>::type>::Ptr intensities = HostPinnedArray<Field<INTENSITY_F32>::type>::create();
};
