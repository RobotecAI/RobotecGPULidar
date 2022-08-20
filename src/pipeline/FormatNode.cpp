#include <pipeline/Nodes.hpp>

std::shared_ptr<const VArray> FormatNode::getData() const
{
	return std::shared_ptr<const VArray>();
}

std::size_t FormatNode::getElemSize() const
{
	return 0;
}

void FormatNode::validate()
{

}

void FormatNode::schedule(cudaStream_t stream)
{

}
