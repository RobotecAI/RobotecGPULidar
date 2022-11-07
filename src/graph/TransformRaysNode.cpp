#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

void TransformRaysNode::validate()
{
	input = getValidInput<IRaysNode>();
}

void TransformRaysNode::schedule(cudaStream_t stream)
{
	rays->resize(getRayCount());
	gpuTransformRays(stream, getRayCount(), input->getRays()->getDevicePtr(), rays->getDevicePtr(), transform);
}
