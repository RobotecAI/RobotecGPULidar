#include <pipeline/Nodes.hpp>

void UseRaysMat3x4fNode::setParameters(const Mat3x4f *raysRaw, size_t rayCount)
{
	rays->copyFrom(raysRaw, rayCount);
}

void UseRaysMat3x4fNode::validate()
{
	rays->hintLocation(VArray::GPU);
}
