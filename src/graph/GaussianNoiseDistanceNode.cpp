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

#include <gpu/helpersKernels.hpp>
#include <gpu/gaussianNoiseKernels.hpp>
#include <graph/NodesCore.hpp>

void GaussianNoiseDistanceNode::setParameters(float mean, float stDevBase, float stDevRisePerMeter)
{
	this->mean = mean;
	this->stDevBase = stDevBase;
	this->stDevRisePerMeter = stDevRisePerMeter;
}

void GaussianNoiseDistanceNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();
	lookAtOriginTransform = input->getLookAtOriginTransform();

	// This node will modifty field DISTANCE_F32 if present.
	// In the future: only one field should be modified.
	// Other fields that depend on the main field (for now, it's XYZ_F32) should be calculated somewhere else (e.g., in data getters nodes).
	if (input->hasField(DISTANCE_F32)) {
		if (outDistance == nullptr) {
			outDistance = DeviceAsyncArray<Field<DISTANCE_F32>::type>::create(arrayMgr);
		}
	} else {
		outDistance.reset();
	}
}

void GaussianNoiseDistanceNode::enqueueExecImpl()
{
	auto pointCount = input->getPointCount();
	outXyz->resize(pointCount, false, false);

	Field<DISTANCE_F32>::type* outDistancePtr = nullptr;
	if (outDistance != nullptr) {
		outDistance->resize(pointCount, false, false);
		outDistancePtr = outDistance->getWritePtr();
	}

	if (randomizationStates->getCount() < pointCount) {
		randomizationStates->resize(pointCount, false, false);
                gpuSetupRandomNumberGenerator(getStreamHandle(), pointCount, randomDevice(), randomizationStates->getWritePtr());
	}

	const auto* inXyzPtr = input->getFieldDataTyped<XYZ_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto* outXyzPtr = outXyz->getWritePtr();
	auto* randPtr = randomizationStates->getWritePtr();
	gpuAddGaussianNoiseDistance(getStreamHandle(), pointCount, mean, stDevBase, stDevRisePerMeter, lookAtOriginTransform, randPtr, inXyzPtr, outXyzPtr, outDistancePtr);
}

IAnyArray::ConstPtr GaussianNoiseDistanceNode::getFieldData(rgl_field_t field)
{
	if (field == XYZ_F32) {
		return outXyz;
	}
	if (field == DISTANCE_F32 && outDistance != nullptr) {
		return outDistance;
	}
	return input->getFieldData(field);
}
