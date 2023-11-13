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
#include <graph/GraphRunCtx.hpp>

void CompactPointsNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();
	// Needed to clear cache because fields in the pipeline may have changed
	// In fact, the cache manager is no longer useful here
	// To be kept/removed in some future refactor (when resolving comment in the `enqueueExecImpl`)
	cacheManager.clear();
}

void CompactPointsNode::enqueueExecImpl()
{
	cacheManager.trigger();
	inclusivePrefixSum->resize(input->getHeight() * input->getWidth(), false, false);
	size_t pointCount = input->getWidth() * input->getHeight();
	const auto* isHit = input->getFieldDataTyped<IS_HIT_I32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	if (pointCount > 0) {
		gpuFindCompaction(getStreamHandle(), pointCount, isHit, inclusivePrefixSum->getWritePtr(), &width);
	}

	// getFieldData may be called in client's thread from rgl_graph_get_result_data
	// Doing job there would be:
	// - unexpected (job was supposed to be done asynchronously)
	// - hard to implement:
	//     - to avoid blocking on yet-running graph stream, we would need do it in copy stream, which would require
	//       temporary rebinding DAAs to copy stream, which seems like nightmarish idea
	// Therefore, once we know what fields are requested, we compute them eagerly
	// This is supposed to be removed in some future refactor (e.g. when introducing LayeredSoA)
	for (auto&& field : cacheManager.getKeys()) {
		getFieldData(field);
	}
}

IAnyArray::ConstPtr CompactPointsNode::getFieldData(rgl_field_t field)
{
	std::lock_guard lock{getFieldDataMutex};

	if (!cacheManager.contains(field)) {
		auto fieldData = createArray<DeviceAsyncArray>(field, arrayMgr);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(width, false, false);
		if (width > 0) {
			char* outPtr = static_cast<char*>(fieldData->getRawWritePtr());
			if (!isDeviceAccessible(input->getFieldData(field)->getMemoryKind())) {
				auto msg = fmt::format("CompactPointsNode requires its input to be device-accessible, {} is not", field);
				throw InvalidPipeline(msg);
			}
			const char* inputPtr = static_cast<const char*>(input->getFieldData(field)->getRawReadPtr());
			const auto* isHitPtr = input->getFieldDataTyped<IS_HIT_I32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
			const CompactionIndexType* indices = inclusivePrefixSum->getReadPtr();
			gpuApplyCompaction(getStreamHandle(), input->getPointCount(), getFieldSize(field), isHitPtr, indices, outPtr,
			                   inputPtr);
			bool calledFromEnqueue = graphRunCtx.value()->isThisThreadGraphThread();
			if (!calledFromEnqueue) {
				// This is a special case, where API calls getFieldData for this field for the first time
				// We did not enqueued compcation in enqueueExecImpl, yet, we are asked for results.
				// This operation was enqueued in the graph stream, but API won't wait for whole graph stream.
				// Therefore, we need a manual sync here.
				// TODO: remove this cancer.
				CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));
			}
		}
		cacheManager.setUpdated(field);
	}

	return std::const_pointer_cast<const IAnyArray>(cacheManager.getValue(field));
}

size_t CompactPointsNode::getWidth() const
{
	this->synchronize();
	return width;
}
