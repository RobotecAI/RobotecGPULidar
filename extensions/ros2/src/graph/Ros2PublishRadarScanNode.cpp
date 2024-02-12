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

void Ros2PublishRadarScanNode::setParameters(const char* topicName, const char* frameId,
                                             rgl_qos_policy_reliability_t qosReliability,
                                             rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory,
                                             int32_t qosHistoryDepth)
{
	ros2InitGuard = Ros2InitGuard::acquire();

	ros2Message.header.frame_id = frameId;

	auto qos = rclcpp::QoS(qosHistoryDepth);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(qosReliability));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(qosDurability));
	qos.history(static_cast<rmw_qos_history_policy_t>(qosHistory));
	ros2Publisher = ros2InitGuard->createUniquePublisher<radar_msgs::msg::RadarScan>(topicName, qos);
}

void Ros2PublishRadarScanNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();
	if (input->getHeight() != 1) {
		throw InvalidPipeline("ROS2 radar publish supports unorganized pointclouds only");
	}
}

void Ros2PublishRadarScanNode::ros2EnqueueExecImpl()
{
	ros2Message.header.stamp = Scene::instance().getTime().has_value() ?
	                               Scene::instance().getTime().value().asRos2Msg() :
	                               static_cast<builtin_interfaces::msg::Time>(ros2InitGuard->getNode().get_clock()->now());
	std::vector<rgl_field_t> fields = this->getRequiredFieldList();
	FormatPointsNode::formatAsync(formattedData, input, fields, fieldDescBuilder);
	ros2Message.returns.resize(input->getPointCount());
	CHECK_CUDA(cudaMemcpyAsync(ros2Message.returns.data(), formattedData->getReadPtr(),
	                           formattedData->getSizeOf() * formattedData->getCount(), cudaMemcpyDeviceToHost,
	                           formattedData->getStream()->getHandle()));
	CHECK_CUDA(cudaStreamSynchronize(formattedData->getStream()->getHandle()));
	ros2Publisher->publish(ros2Message);
}
