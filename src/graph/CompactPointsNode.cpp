// Copyright 2022 Robotec.AI
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
#include <RGLFields.hpp>
#include <repr.hpp>

// TODO: WritePCD triggers cudaSynchronizeStream in its indirect input CompactNode
// TODO: This can be alleviated with a stream-aware VArray :)

void CompactPointsNode::enqueueExecImpl()
{
	cacheManager.trigger();
	inclusivePrefixSum->resize(input->getHeight() * input->getWidth(), false, false);
	size_t pointCount = input->getWidth() * input->getHeight();
	const auto* isHit = input->getFieldDataTyped<IS_HIT_I32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	gpuFindCompaction(getStreamHandle(), pointCount, isHit, inclusivePrefixSum->getWritePtr(), &width);
	CHECK_CUDA(cudaEventRecord(finishedEvent, getStreamHandle()));
}

IAnyArray::ConstPtr CompactPointsNode::getFieldData(rgl_field_t field)
{
	CHECK_CUDA(cudaEventSynchronize(finishedEvent));
	if (!cacheManager.contains(field)) {
		auto fieldData = createArray<DeviceAsyncArray>(field, arrayMgr);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(width, false, false);
		char* outPtr = static_cast<char *>(fieldData->getRawWritePtr());

		if (!isDeviceAccessible(input->getFieldData(field)->getMemoryKind())) {
			auto msg = fmt::format("CompactPointsNode requires its input to be device-accessible, {} is not", field);
			throw InvalidPipeline(msg);
		}
		const char* inputPtr = static_cast<const char *>(input->getFieldData(field)->getRawReadPtr());
		const auto* isHitPtr = input->getFieldDataTyped<IS_HIT_I32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
		const CompactionIndexType * indices = inclusivePrefixSum->getReadPtr();
		gpuApplyCompaction(getStreamHandle(), input->getPointCount(), getFieldSize(field), isHitPtr, indices, outPtr, inputPtr);
		CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));
		cacheManager.setUpdated(field);
	}

	return std::const_pointer_cast<const IAnyArray>(cacheManager.getValue(field));
}

size_t CompactPointsNode::getWidth() const
{
	CHECK_CUDA(cudaEventSynchronize(finishedEvent));
	return width;
}
