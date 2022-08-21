#include <pipeline/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

void TransformRaysNode::validate(cudaStream_t stream)
{
	input = getValidInput<IRaysNode>();
	rays->resize(getRayCount());
	rays->hintLocation(VArray::GPU, stream);
}

void TransformRaysNode::schedule(cudaStream_t stream)
{
	gpuTransformRays(stream, getRayCount(), input->getRays()->getDevicePtr(), rays->getDevicePtr(), transform);
}
