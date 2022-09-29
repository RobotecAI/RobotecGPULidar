#include <graph/Nodes.hpp>

void YieldPointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot yield field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}
	this->fields = fields;
}

void YieldPointsNode::schedule(cudaStream_t stream)
{
	for (auto&& field : fields) {
		results[field] = input->getFieldData(field, stream);
	}
}
