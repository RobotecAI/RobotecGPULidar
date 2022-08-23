#include <pipeline/Nodes.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>
#include <repr.hpp>

// TODO: WritePCD triggers cudaSynchronizeStream in its indirect input CompactNode
// TODO: This can be alleviated with a stream-aware VArray :)

void CompactNode::validate(cudaStream_t stream)
{
	input = getValidInput<IPointCloudNode>();
	inclusivePrefixSum->resize(input->getHeight() * input->getWidth());
	inclusivePrefixSum->hintLocation(VArray::GPU, stream);
}

void CompactNode::schedule(cudaStream_t stream)
{
	this->stream = stream;
	size_t pointCount = input->getWidth() * input->getHeight();
	const auto* isHit = input->getFieldData(RGL_FIELD_IS_HIT_I32, stream)->getTypedProxy<RGLField<RGL_FIELD_IS_HIT_I32>::Type>()->getDevicePtr();
	gpuFindCompaction(stream, pointCount, isHit, inclusivePrefixSum->getDevicePtr(), &width);
	CHECK_CUDA(cudaStreamSynchronize(stream));
}

VArray::ConstPtr CompactNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	VArray::Ptr out = VArray::create(field, width);
	char* outPtr = static_cast<char *>(out->getDevicePtr());
	const char* inputPtr = static_cast<const char *>(input->getFieldData(field, stream)->getDevicePtr());
	const auto* isHitPtr = input->getFieldData(RGL_FIELD_IS_HIT_I32, stream)->getTypedProxy<RGLField<RGL_FIELD_IS_HIT_I32>::Type>()->getDevicePtr();
	const CompactionIndexType * indices = inclusivePrefixSum->getDevicePtr();
	gpuApplyCompaction(stream, input->getPointCount(), getFieldSize(field), isHitPtr, indices, outPtr, inputPtr);
	return out;
}

size_t CompactNode::getWidth() const
{
	CHECK_CUDA(cudaStreamSynchronize(stream));
	return width;
}
