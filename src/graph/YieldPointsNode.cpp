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

void YieldPointsNode::validate()
{
	input = getValidInput<IPointsNode>();
	for (auto&& field : fields) {
		if (!input->hasField(field)) {
			auto msg = fmt::format("YieldPointsNode's input does not provide required field {}", toString(field));
			throw InvalidPipeline(msg);
		}
	}
}
