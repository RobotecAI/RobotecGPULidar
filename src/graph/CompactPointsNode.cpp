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

void CompactPointsNode::schedule(cudaStream_t stream)
{
	cacheManager.trigger();
	inclusivePrefixSum->resize(input->getHeight() * input->getWidth(), false, false);
	size_t pointCount = input->getWidth() * input->getHeight();
	const auto* isHit = input->getFieldDataTyped<IS_HIT_I32>(stream)->getDevicePtr();
	gpuFindCompaction(stream, pointCount, isHit, inclusivePrefixSum->getDevicePtr(), &width);
	CHECK_CUDA(cudaEventRecord(finishedEvent, stream));
}

VArray::ConstPtr CompactPointsNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	CHECK_CUDA(cudaEventSynchronize(finishedEvent));
	if (!cacheManager.contains(field)) {
		auto fieldData = VArray::create(field, width);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(width, false, false);
		char* outPtr = static_cast<char *>(fieldData->getWritePtr(MemLoc::Device));
		const char* inputPtr = static_cast<const char *>(input->getFieldData(field, stream)->getReadPtr(MemLoc::Device));
		const auto* isHitPtr = input->getFieldDataTyped<IS_HIT_I32>(stream)->getDevicePtr();
		const CompactionIndexType * indices = inclusivePrefixSum->getDevicePtr();
		gpuApplyCompaction(stream, input->getPointCount(), getFieldSize(field), isHitPtr, indices, outPtr, inputPtr);
		CHECK_CUDA(cudaStreamSynchronize(stream));
		cacheManager.setUpdated(field);
	}

	return std::const_pointer_cast<const VArray>(cacheManager.getValue(field));
}

size_t CompactPointsNode::getWidth() const
{
	CHECK_CUDA(cudaEventSynchronize(finishedEvent));
	return width;
}
