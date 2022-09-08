#include <graph/Node.hpp>

API_OBJECT_INSTANCE(Node);

void Node::addParent(Node::Ptr parent)
{
	if (parent == nullptr) {
		auto msg = fmt::format("attempted to set an empty parent for {}", getName());
		throw InvalidPipeline(msg);
	}
	auto parentIt = std::find(this->inputs.begin(), this->inputs.end(), parent);
	if (parentIt != this->inputs.end()) {
		auto msg = fmt::format("attempted to add parent {} to child {} twice", parent->getName(), getName());
		throw InvalidPipeline(msg);
	}
	this->inputs.push_back(parent);
	parent->outputs.push_back(shared_from_this());
}

void Node::addChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to set an empty child for {}", getName());
		throw InvalidPipeline(msg);
	}
	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	if (childIt != this->outputs.end()) {
		auto msg = fmt::format("attempted to add child {} to parent {} twice", child->getName(), getName());
		throw InvalidPipeline(msg);
	}
	this->outputs.push_back(child);
	child->inputs.push_back(shared_from_this());
}

void Node::removeChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to remove an empty child for {}", getName());
		throw InvalidPipeline(msg);
	}
	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	if (childIt == this->outputs.end()) {
		auto msg = fmt::format("attempted to remove child {} from {},"
		                       "but it was not found", child->getName(), getName());
		throw InvalidPipeline(msg);
	}
	// Remove child from our children
	this->outputs.erase(childIt);

	auto thisIt = std::find(child->inputs.begin(), child->inputs.end(), shared_from_this());
	if (thisIt == child->inputs.end()) {
		auto msg = fmt::format("attempted to remove parent {} from {},"
		                       "but it was not found", getName(), child->getName());
		throw InvalidPipeline(msg);
	}
	// Remove us as a parent of that child
	child->inputs.erase(thisIt);
}

void Node::prependNode(Node::Ptr node)
{
	for (auto&& in : inputs) {
		// Remove us as a child of our parents ;)
		auto thisIt = std::find(in->outputs.begin(), in->outputs.end(), shared_from_this());
		if (thisIt != in->outputs.end()) {
			in->outputs.erase(thisIt);  // It is invalid to call erase() on end()
		}
		node->addParent(in);
	}
	inputs.clear();
	addParent(node);
}
