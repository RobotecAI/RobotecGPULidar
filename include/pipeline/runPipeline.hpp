#include <pipeline/Node.hpp>

void validatePipeline(std::shared_ptr<Node> root)
{
	root->validate();
	for (auto&& child : root->children) {
		validatePipeline(child);
	}
}

void runPipeline(std::shared_ptr<Node> node)
{
	while (node->parent != nullptr) {
		node = node->parent;
	}

	validatePipeline(node);
}