#include <pipeline/Node.hpp>

void runPipeline(std::shared_ptr<Node> node)
{
	while (node->parent == nullptr) {
		node = node->parent;
	}

	// Be brave!
}