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

void GaussianNoiseAngularRaysNode::setParameters(float mean, float stDev, rgl_axis_t rotationAxis)
{
	this->mean = mean;
	this->stDev = stDev;
	this->rotationAxis = rotationAxis;
}

void GaussianNoiseAngularRaysNode::validateImpl()
{
	IRaysNodeSingleInput::validateImpl();
	lookAtOriginTransform = input->getCumulativeRayTransfrom().inverse();

	auto rayCount = input->getRayCount();
	rays->resize(rayCount, false, false);

	if (randomizationStates->getCount() < rayCount) {
		randomizationStates->resize(rayCount, false, false);
		gpuSetupGaussianNoiseGenerator(getStreamHandle(), rayCount, randomDevice(), randomizationStates->getWritePtr());
	}
}

void GaussianNoiseAngularRaysNode::enqueueExecImpl()
{
	const auto* inRaysPtr = input->getRays()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto* outRaysPtr = rays->getWritePtr();
	auto* randPtr = randomizationStates->getWritePtr();
	gpuAddGaussianNoiseAngularRay(getStreamHandle(), getRayCount(), mean, stDev, rotationAxis, lookAtOriginTransform, randPtr,
	                              inRaysPtr, outRaysPtr);
}
