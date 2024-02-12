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

#include <graph/NodesRos2.hpp>
#include <scene/Scene.hpp>

void Ros2PublishPointVelocityMarkersNode::setParameters(const char* topicName, const char* frameId, rgl_field_t velocityField)
{
	if (velocityField != RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32 && velocityField != RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32) {
		auto msg = fmt::format("{} cannot publish fields other than `RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32` and "
		                       "`RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32` (`{}` was requested)",
		                       getName(), toString(velocityField));
		throw InvalidAPIArgument(msg);
	}
	this->frameId = frameId;
	ros2InitGuard = Ros2InitGuard::acquire();
	auto qos = rclcpp::QoS(10); // Use system default QoS
	linesPublisher = ros2InitGuard->createUniquePublisher<visualization_msgs::msg::Marker>(topicName, qos);
	this->velocityField = velocityField;
}

void Ros2PublishPointVelocityMarkersNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();
	if (!input->isDense()) {
		throw InvalidPipeline(fmt::format("{} requires a compacted point cloud (dense)", getName()));
	}
}
void Ros2PublishPointVelocityMarkersNode::ros2EnqueueExecImpl()
{
	pos->copyFrom(input->getFieldData(RGL_FIELD_XYZ_VEC3_F32));
	vel->copyFrom(input->getFieldData(velocityField));

	if (!rclcpp::ok()) {
		// TODO: This should be handled by the Graph.
		throw std::runtime_error("Unable to publish a message because ROS2 has been shut down.");
	}
	linesPublisher->publish(makeLinesMarker());
}

const visualization_msgs::msg::Marker& Ros2PublishPointVelocityMarkersNode::makeLinesMarker()
{
	marker.header.stamp = Scene::instance().getTime().has_value() ?
	                          Scene::instance().getTime().value().asRos2Msg() :
	                          static_cast<builtin_interfaces::msg::Time>(ros2InitGuard->getNode().get_clock()->now());
	marker.header.frame_id = this->frameId;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 0.32;
	marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	marker.scale.x = 0.02; // Line width diameter
	marker.points.resize(pos->getCount() * 2);
	geometry_msgs::msg::Point origin, end;
	for (int i = 0; i < pos->getCount(); i++) {
		origin.x = pos->at(i).x();
		origin.y = pos->at(i).y();
		origin.z = pos->at(i).z();
		end.x = origin.x + vel->at(i).x();
		end.y = origin.y + vel->at(i).y();
		end.z = origin.z + vel->at(i).z();
		marker.points[2 * i] = origin;
		marker.points[2 * i + 1] = end;
	}
	return marker;
}