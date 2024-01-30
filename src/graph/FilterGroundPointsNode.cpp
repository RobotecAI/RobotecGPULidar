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

void FilterGroundPointsNode::setParameters(const rgl_vec3f* sensor_up_vector, float ground_angle_threshold)
{
	this->sensor_up_vector = sensor_up_vector;
	this->ground_angle_threshold = ground_angle_threshold;
}

void FilterGroundPointsNode::validateImpl() { IPointsNodeSingleInput::validateImpl(); }

void FilterGroundPointsNode::enqueueExecImpl()
{
	auto pointCount = input->getPointCount();
	ouNonGround->resize(pointCount, false, false);

	const auto* inXyzPtr = input->getFieldDataTyped<XYZ_VEC3_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	const auto* inNormalsPtr = input->getFieldDataTyped<NORMAL_VEC3_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto lidarTransform = input->getLookAtOriginTransform();

	gpuFilterGroundPoints(getStreamHandle(), pointCount, sensor_up_vector, ground_angle_threshold, inXyzPtr, inNormalsPtr,
	                      outNonGround->getWritePtr(), lidarTransform);
}

IAnyArray::ConstPtr FilterGroundPointsNode::getFieldData(rgl_field_t field)
{
	if (field == IS_GROUND_I32) {
		return ouNonGround;
	}

	return input->getFieldData(field);
}
