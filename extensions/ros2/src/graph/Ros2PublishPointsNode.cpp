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

#include <graph/NodesRos2.hpp>
#include <scene/Scene.hpp>
#include <RGLFields.hpp>

void Ros2PublishPointsNode::setParameters(const char* topicName, const char* frameId,
                                          rgl_qos_policy_reliability_t qosReliability,
                                          rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory,
                                          int32_t qosHistoryDepth)
{
	ros2InitGuard = Ros2InitGuard::acquire();

	ros2Message.header.frame_id = frameId;

	rclcpp::QoS qos = rclcpp::QoS(qosHistoryDepth);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(qosReliability));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(qosDurability));
	qos.history(static_cast<rmw_qos_history_policy_t>(qosHistory));
	ros2Publisher = ros2InitGuard->createUniquePublisher<sensor_msgs::msg::PointCloud2>(topicName, qos);
}

void Ros2PublishPointsNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();
	if (input->getHeight() != 1) {
		throw InvalidPipeline("ROS2 publish support unorganized pointclouds only");
	}
	updateRos2Message(input->getRequiredFieldList(), input->isDense());
}

void Ros2PublishPointsNode::enqueueExecImpl()
{
	auto fieldData = input->getFieldData(RGL_FIELD_DYNAMIC_FORMAT)->asTyped<char>()->asSubclass<HostArray>();
	int count = input->getPointCount();
	ros2Message.data.resize(ros2Message.point_step * count);
	const void* src = fieldData->getRawReadPtr();
	size_t size = fieldData->getCount() * fieldData->getSizeOf();
	CHECK_CUDA(cudaMemcpyAsync(ros2Message.data.data(), src, size, cudaMemcpyDefault, getStreamHandle()));
	CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));
	ros2Message.width = count;
	ros2Message.row_step = ros2Message.point_step * ros2Message.width;
	// TODO(msz-rai): Assign scene to the Graph.
	// For now, only default scene is supported.
	ros2Message.header.stamp = Scene::instance().getTime().has_value() ?
	                               Scene::instance().getTime()->asRos2Msg() :
	                               static_cast<builtin_interfaces::msg::Time>(ros2InitGuard->node->get_clock()->now());
	if (!rclcpp::ok()) {
		throw std::runtime_error("Unable to publish a message because ROS2 has been shut down.");
	}
	ros2Publisher->publish(ros2Message);
}


void Ros2PublishPointsNode::updateRos2Message(const std::vector<rgl_field_t>& fields, bool isDense)
{
	ros2Message = sensor_msgs::msg::PointCloud2();
	int offset = 0;
	for (const auto& field : fields) {
		auto ros2fields = toRos2Fields(field);
		auto ros2names = toRos2Names(field);
		auto ros2sizes = toRos2Sizes(field);

		for (int i = 0; i < ros2sizes.size(); ++i) {
			if (ros2fields.size() > i && ros2names.size() > i) {
				ros2Message.fields.push_back([&] {
					auto ret = sensor_msgs::msg::PointField();
					ret.name = ros2names[i];
					ret.datatype = ros2fields[i];
					ret.offset = offset;
					ret.count = 1;
					return ret;
				}());
			}
			offset += ros2sizes[i];
		}
	}
	ros2Message.height = 1;
	ros2Message.point_step = offset;
	ros2Message.is_dense = isDense;
	ros2Message.is_bigendian = false;
}
