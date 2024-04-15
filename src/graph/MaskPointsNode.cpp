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
#include <gpu/nodeKernels.hpp>

void MaskPointsNode::setParameters(const int* maskRaw, size_t maskPointCount)
{
	pointCount = maskPointCount;
	pointsMask->copyFromExternal(maskRaw, maskPointCount);
}

void MaskPointsNode::enqueueExecImpl()
{
	auto inputPointCount = input->getWidth() * input->getHeight();

	if (inputPointCount != pointCount) {
		throw InvalidPipeline("MaskPointsNode: input point count does not match mask point count");
	}

	output->resize(inputPointCount, false, false);
	const auto inputField = input->getFieldDataTyped<IS_HIT_I32>()->asSubclass<DeviceAsyncArray>();
	auto* outputPtr = output->getWritePtr();

	gpuMaskPoints(getStreamHandle(), pointCount, pointsMask->getReadPtr(), inputField->getReadPtr(), outputPtr);
}

IAnyArray::ConstPtr MaskPointsNode::getFieldData(rgl_field_t field)
{
	if (field == IS_HIT_I32) {
		return output;
	}
	return input->getFieldData(field);
}