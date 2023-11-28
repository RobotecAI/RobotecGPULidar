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

void Ros2PublishPointVelocityMarkersNode::setParameters(const char* topicName)
{
	ros2InitGuard = Ros2InitGuard::acquire();
	linesPublisher = ros2InitGuard->createUniquePublisher<visualization_msgs::msg::MarkerArray>(topicName, 8);
}

void Ros2PublishPointVelocityMarkersNode::validateImpl()
{

	IPointsNodeSingleInput::validateImpl();
	bool acceptableInput = input->isDense();
	for (auto&& field : getRequiredFieldList()) {
		acceptableInput &= input->hasField(field);
	}
	if (!acceptableInput) {
		throw InvalidPipeline("Ros2PublishPointVelocityMarkersNode: invalid input");
	}
}
void Ros2PublishPointVelocityMarkersNode::enqueueExecImpl()
{
	pos->copyFrom(input->getFieldData(RGL_FIELD_XYZ_VEC3_F32));
	vel->copyFrom(input->getFieldData(RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32));

	// Add special marker to delete previous markers
	auto cancelMsg = visualization_msgs::msg::Marker();
	cancelMsg.action = visualization_msgs::msg::Marker::DELETEALL;
	cancelMsg.id = -1;

	visualization_msgs::msg::MarkerArray lines;
	lines.markers.emplace_back(cancelMsg);
	lines.markers.emplace_back(std::move(makeLinesMarker()));
	linesPublisher->publish(lines);
}

visualization_msgs::msg::Marker Ros2PublishPointVelocityMarkersNode::makeLinesMarker()
{
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "world";
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 0.16;
	marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	marker.scale.x = 0.02; // Line width diameter
	marker.id = 0;
	marker.header.stamp = Scene::instance().getTime().value().asRos2Msg();
	geometry_msgs::msg::Point origin, end;
	marker.points.reserve(pos->getCount() * 2);
	for (int i = 0; i < pos->getCount(); i++) {
		origin.x = pos->at(i).x();
		origin.y = pos->at(i).y();
		origin.z = pos->at(i).z();
		end.x = origin.x + vel->at(i).x();
		end.y = origin.y + vel->at(i).y();
		end.z = origin.z + vel->at(i).z();
		marker.points.emplace_back(origin);
		marker.points.emplace_back(end);
	}
	return marker;
}