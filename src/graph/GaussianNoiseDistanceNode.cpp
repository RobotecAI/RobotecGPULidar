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

#include <graph/NodesCore.hpp>
#include <gpu/gaussianNoiseKernels.hpp>
#include <gpu/helpersKernels.hpp>

void GaussianNoiseDistanceNode::setParameters(float mean, float stDevBase, float stDevRisePerMeter)
{
	this->mean = mean;
	this->stDevBase = stDevBase;
	this->stDevRisePerMeter = stDevRisePerMeter;
}

void GaussianNoiseDistanceNode::enqueueExecImpl()
{
	auto pointCount = input->getPointCount();
	outXyz->resize(pointCount, false, false);
	outDistance->resize(pointCount, false, false);

	if (randomizationStates->getCount() < pointCount) {
		randomizationStates->resize(pointCount, false, false);
		gpuSetupRandomNumberGenerator(getStreamHandle(), pointCount, randomDevice(), randomizationStates->getWritePtr());
	}

	const auto* inXyzPtr = input->getFieldDataTyped<XYZ_VEC3_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	const auto* inDistancePtr = input->getFieldDataTyped<DISTANCE_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto* outXyzPtr = outXyz->getWritePtr();
	auto* outDistancePtr = outDistance->getWritePtr();
	auto* randPtr = randomizationStates->getWritePtr();
	gpuAddGaussianNoiseDistance(getStreamHandle(), pointCount, mean, stDevBase, stDevRisePerMeter,
	                            input->getLookAtOriginTransform(), randPtr, inXyzPtr, inDistancePtr, outXyzPtr, outDistancePtr);
}

IAnyArray::ConstPtr GaussianNoiseDistanceNode::getFieldData(rgl_field_t field)
{
	if (field == XYZ_VEC3_F32) {
		return outXyz;
	}
	if (field == DISTANCE_F32) {
		return outDistance;
	}
	return input->getFieldData(field);
}
