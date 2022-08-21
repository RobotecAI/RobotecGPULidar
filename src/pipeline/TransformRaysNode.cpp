#include <pipeline/Nodes.hpp>

void TransformRaysNode::validate(cudaStream_t stream)
{
	input = getValidInput<IRaysNode>();
}

void TransformRaysNode::schedule(cudaStream_t stream)
{

}
