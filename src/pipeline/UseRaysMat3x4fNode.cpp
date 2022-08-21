#include <pipeline/Nodes.hpp>

void UseRaysMat3x4fNode::setParameters(const Mat3x4f *raysRaw, size_t rayCount)
{
	rays = VArrayProxy<Mat3x4f>::create();
	rays->copyFrom(raysRaw, rayCount);
}

void UseRaysMat3x4fNode::validate(cudaStream_t stream)
{
	rays->hintLocation(VArray::GPU, stream);
}
