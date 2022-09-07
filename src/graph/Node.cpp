#include <graph/Node.hpp>

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

void Node::addChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to set an empty child for {}", getName());
		throw InvalidPipeline(msg);
	}
	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	if (childIt != this->outputs.end()) {
		auto msg = fmt::format("attempted to set an child "
		                       "for {} that already is his child", getName());
		throw InvalidPipeline(msg);
	}
	this->outputs.push_back(child);
	if (child != nullptr) {
		child->inputs.push_back(shared_from_this());
	}
}

void Node::removeChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to remove an empty child for {}", getName());
		throw InvalidPipeline(msg);
	}
	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	if (childIt == this->outputs.end()) {
		auto msg = fmt::format("attempted to remove an child "
		                       "from {} that isn't his child", getName());
		throw InvalidPipeline(msg);
	} else {
		// Remove child from our children
		this->outputs.erase(childIt);
	}
	// Remove us as a parent of that child
	auto thisIt = std::find(child->inputs.begin(), child->inputs.end(), shared_from_this());
	if (thisIt != child->inputs.end()) {
		child->inputs.erase(thisIt);
	}
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
