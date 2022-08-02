#include <graph/Nodes.hpp>

void YieldPointsNode::schedule(cudaStream_t stream)
{
	for (auto&& field : fields) {
		results[field] = input->getFieldData(field, stream);
	}
}
