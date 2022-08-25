#include <pipeline/Nodes.hpp>

void TransformPointsNode::validate()
{

}

void TransformPointsNode::schedule(cudaStream_t stream)
{

}

VArray::ConstPtr TransformPointsNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	return VArray::ConstPtr();
}
