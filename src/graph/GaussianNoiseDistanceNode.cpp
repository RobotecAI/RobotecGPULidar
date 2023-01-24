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
#include <gpu/gaussianNoiseKernels.hpp>

void GaussianNoiseDistanceNode::setParameters(float mean, float stDevBase, float stDevRisePerMeter)
{
	this->mean = mean;
	this->stDevBase = stDevBase;
	this->stDevRisePerMeter = stDevRisePerMeter;
}

void GaussianNoiseDistanceNode::validate()
{
	input = getValidInput<RaytraceNode>();

	outDistance.reset();
	if (input->hasField(DISTANCE_F32)) {
		outDistance = VArrayProxy<Field<DISTANCE_F32>::type>::create();
	}

	auto pointCount = input->getPointCount();
	randomizationStates->resize(pointCount, false, false);
	gpuSetupGaussianNoiseGenerator(nullptr, pointCount, randomDevice(), randomizationStates->getDevicePtr());
}

void GaussianNoiseDistanceNode::schedule(cudaStream_t stream)
{
	auto pointCount = input->getPointCount();
	outXyz->resize(pointCount, false, false);

	Field<DISTANCE_F32>::type* outDistancePtr = nullptr;
	if (outDistance != nullptr) {
		outDistance->resize(pointCount, false, false);
		outDistancePtr = outDistance->getDevicePtr();
	}

	const auto inXyz = input->getFieldDataTyped<XYZ_F32>(stream);
	const auto* inXyzPtr = inXyz->getDevicePtr();
	auto* outXyzPtr = outXyz->getDevicePtr();
	gpuAddGaussianNoiseDistance(stream, pointCount, mean, stDevBase, stDevRisePerMeter, randomizationStates->getDevicePtr(), inXyzPtr, outXyzPtr, outDistancePtr);
}

VArray::ConstPtr GaussianNoiseDistanceNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	if (field == XYZ_F32) {
		// TODO(msz-rai): check sync is necessary
		CHECK_CUDA(cudaStreamSynchronize(stream));
		return outXyz->untyped();
	}
	if (field == DISTANCE_F32 && outDistance != nullptr) {
		// TODO(msz-rai): check sync is necessary
		CHECK_CUDA(cudaStreamSynchronize(stream));
		return outDistance->untyped();
	}
	return input->getFieldData(field, stream);
}

std::vector<rgl_field_t> GaussianNoiseDistanceNode::getRequiredFieldList() const
{
	return {XYZ_F32};
}
