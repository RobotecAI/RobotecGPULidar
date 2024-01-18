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

#include <graph/NodesCore.hpp>

void FilterGroundPointsNode::setParameters(rgl_axis_t sensor_up_axis, float ground_angle_threshold)
{
	this->sensor_up_axis = sensor_up_axis;
	this->ground_angle_threshold = ground_angle_threshold;
}

void FilterGroundPointsNode::validateImpl() { IPointsNodeSingleInput::validateImpl(); }

void FilterGroundPointsNode::enqueueExecImpl()
{
	auto pointCount = input->getPointCount();
	ouNonGround->resize(pointCount, false, false);

	const auto* inXyzPtr = input->getFieldDataTyped<XYZ_VEC3_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	const auto* inIncidentAnglesPtr = input->getFieldDataTyped<INCIDENT_ANGLE_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto lidarTransform = input->getLookAtOriginTransform();

	gpuFilterGroundPoints(getStreamHandle(), pointCount, sensor_up_axis, ground_angle_threshold, inXyzPtr, inIncidentAnglesPtr,
	                      ouNonGround->getWritePtr(), lidarTransform);
}

IAnyArray::ConstPtr FilterGroundPointsNode::getFieldData(rgl_field_t field)
{
	if (field == IS_GROUND_I32) {
		return ouNonGround;
	}

	return input->getFieldData(field);
}
