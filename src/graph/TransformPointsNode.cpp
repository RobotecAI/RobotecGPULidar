#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

void TransformPointsNode::validate()
{
	input = getValidInput<IPointCloudNode>();
	if (!input->hasField(XYZ_F32)) {
		auto msg = fmt::format("{} requires XYZ to be present", getName());
		throw std::invalid_argument(msg);
	}
}

void TransformPointsNode::schedule(cudaStream_t stream)
{
	auto pointCount = input->getWidth() * input->getHeight();
	output->resize(pointCount);
	const auto inputField = input->getFieldDataTyped<XYZ_F32>(stream);
	const auto* inputPtr = inputField->getDevicePtr();
	auto* outputPtr = output->getDevicePtr();
	gpuTransformPoints(stream, pointCount, inputPtr, outputPtr, transform);
}

VArray::ConstPtr TransformPointsNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	if (field == XYZ_F32) {
		// TODO(prybicki): check sync is necessary
		CHECK_CUDA(cudaStreamSynchronize(stream);
		return output->untyped();
	}
	return input->getFieldData(field, stream);
}
