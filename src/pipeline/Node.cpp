#include <pipeline/Node.hpp>

API_OBJECT_INSTANCE(Node);

void Node::setParent(Node::Ptr parent)
{
	if (parent == nullptr) {
		auto msg = fmt::format("attempted to set an empty parent for {}", getNodeTypeName());
		throw InvalidPipeline(msg);
	}
	this->inputs.push_back(parent);
	if (parent != nullptr) {
		parent->outputs.push_back(shared_from_this());
	}
}
