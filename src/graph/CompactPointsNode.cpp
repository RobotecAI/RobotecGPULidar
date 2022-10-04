#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>
#include <repr.hpp>

// TODO: WritePCD triggers cudaSynchronizeStream in its indirect input CompactNode
// TODO: This can be alleviated with a stream-aware VArray :)

void CompactPointsNode::validate()
{
	input = getValidInput<IPointsNode>();
	if (finishedEvent == nullptr) {
		unsigned flags = cudaEventDisableTiming;  // Provides better performance
		CHECK_CUDA(cudaEventCreate(&finishedEvent, flags));
	}
	inclusivePrefixSum->hintLocation(VArray::GPU);
}

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
	if (!cacheManager.contains(field)) {
		auto fieldData = VArray::create(field, width);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(width, false, false);
		char* outPtr = static_cast<char *>(fieldData->getDevicePtr());
		const char* inputPtr = static_cast<const char *>(input->getFieldData(field, stream)->getDevicePtr());
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
