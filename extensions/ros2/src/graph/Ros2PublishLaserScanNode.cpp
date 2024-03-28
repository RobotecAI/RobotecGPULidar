// Copyright 2024 Robotec.AI
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

#include <graph/NodesRos2.hpp>
#include <scene/Scene.hpp>
#include <RGLFields.hpp>

void Ros2PublishLaserScanNode::setParameters(const char* topicName, const char* frameId,
                                             rgl_qos_policy_reliability_t qosReliability,
                                             rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory,
                                             int32_t qosHistoryDepth)
{
	ros2Message.header.frame_id = frameId;
	auto qos = rclcpp::QoS(qosHistoryDepth);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(qosReliability));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(qosDurability));
	qos.history(static_cast<rmw_qos_history_policy_t>(qosHistory));
	ros2Publisher = ros2InitGuard->createUniquePublisher<sensor_msgs::msg::LaserScan>(topicName, qos);
}

void Ros2PublishLaserScanNode::ros2ValidateImpl()
{
	if (input->getHeight() != 1) {
		throw InvalidPipeline("ROS2 publish supports unorganized pointclouds only");
	}
}

void Ros2PublishLaserScanNode::ros2EnqueueExecImpl()
{
	const auto currentTime = Scene::instance().getTime();
	ros2Message.header.stamp = currentTime.has_value() ?
	                               currentTime.value().asRos2Msg() :
	                               static_cast<builtin_interfaces::msg::Time>(ros2InitGuard->getNode().get_clock()->now());

	const size_t pointCount = input->getPointCount();

	angles->copyFrom(input->getFieldDataTyped<AZIMUTH_F32>());
	auto minAng = angles->at(0);
	auto maxAng = minAng;
	for (size_t i = 1; i < pointCount; ++i) {
		if (angles->at(i) < minAng) {
			minAng = angles->at(i);
		}
		if (angles->at(i) > maxAng) {
			maxAng = angles->at(i);
		}
	}
	ros2Message.angle_min = minAng;
	ros2Message.angle_max = maxAng;
	ros2Message.angle_increment = angles->at(1) - angles->at(0);

	times->copyFrom(input->getFieldDataTyped<TIME_STAMP_F64>());
	ros2Message.time_increment = static_cast<float>(times->at(1) - times->at(0));
	ros2Message.scan_time = static_cast<float>(times->at(pointCount - 1) - times->at(0));

	ranges->copyFrom(input->getFieldDataTyped<DISTANCE_F32>());
	ros2Message.ranges.resize(input->getPointCount());
	size_t size = pointCount * Field<DISTANCE_F32>::size;
	CHECK_CUDA(cudaMemcpyAsync(ros2Message.ranges.data(), ranges->getRawReadPtr(), size, cudaMemcpyDefault, getStreamHandle()));
	CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));

	auto minRange = ranges->at(0);
	auto maxRange = minRange;
	for (size_t i = 1; i < pointCount; ++i) {
		if (ranges->at(i) < minRange) {
			minRange = ranges->at(i);
		}
		if (ranges->at(i) > maxRange) {
			maxRange = ranges->at(i);
		}
	}
	ros2Message.range_min = minRange;
	ros2Message.range_max = maxRange;

	intensities->copyFrom(input->getFieldDataTyped<INTENSITY_F32>());
	ros2Message.intensities.resize(pointCount);
	size = pointCount * Field<INTENSITY_F32>::size;
	CHECK_CUDA(cudaMemcpyAsync(ros2Message.intensities.data(), intensities->getRawReadPtr(), size, cudaMemcpyDefault,
	                           getStreamHandle()));
	CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));

	ros2Publisher->publish(ros2Message);
}
