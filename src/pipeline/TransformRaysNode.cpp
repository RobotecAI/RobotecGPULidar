#include <pipeline/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

void TransformRaysNode::validate()
{
	input = getValidInput<IRaysNode>();
	rays->hintLocation(VArray::GPU);
}

void TransformRaysNode::schedule(cudaStream_t stream)
{
	rays->resize(getRayCount());
	gpuTransformRays(stream, getRayCount(), input->getRays()->getDevicePtr(), rays->getDevicePtr(), transform);
}
