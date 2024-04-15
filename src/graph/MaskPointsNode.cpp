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

	const auto inputField = input->getFieldDataTyped<XYZ_VEC3_F32>()->asSubclass<DeviceAsyncArray>();
	const auto* inputPtr = inputField->getReadPtr();
	auto* outputPtr = output->getWritePtr();

	gpuMaskPoints(getStreamHandle(), pointCount, pointsMask->getReadPtr(), inputPtr, outputPtr);
}

IAnyArray::ConstPtr MaskPointsNode::getFieldData(rgl_field_t field)
{
	if (field == IS_HIT_I32) {
		return output;
	}
	return input->getFieldData(field);
}