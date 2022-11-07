#include <graph/Nodes.hpp>

void SetRingIdsRaysNode::setParameters(const int* ringIdsRaw, size_t ringIdsCount)
{
	ringIds->setData(ringIdsRaw, ringIdsCount);
}

void SetRingIdsRaysNode::validate()
{
	input = getValidInput<IRaysNode>();

	if (input->getRayCount() % ringIds->getCount() != 0) {
		auto msg = fmt::format("ring ids doesn't match number of rays. "
		    "RayCount({}) mod RingIdsCount({}) should be zero", input->getRayCount(), ringIds->getCount());
		throw InvalidPipeline(msg);
	}
}
