#include <pipeline/Node.hpp>

API_OBJECT_INSTANCE(Node);

void Node::addParent(Node::Ptr parent)
{
	if (parent == nullptr) {
		auto msg = fmt::format("attempted to set an empty parent for {}", getName());
		throw InvalidPipeline(msg);
	}
	this->inputs.push_back(parent);
	if (parent != nullptr) {
		parent->outputs.push_back(shared_from_this());
	}
}

void Node::prependNode(Node::Ptr node)
{
	for (auto&& in : inputs) {
		// Remove us as a child of our parents ;)
		in->outputs.erase(std::find(in->outputs.begin(), in->outputs.end(), shared_from_this()));
		node->addParent(in);
	}
	inputs.clear();
	addParent(node);
}
