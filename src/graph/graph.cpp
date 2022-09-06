#include <set>
#include <vector>
#include <graph/graph.hpp>
#include <graph/Nodes.hpp>
#include <RGLFields.hpp>

static std::set<Node::Ptr> findConnectedNodes(Node::Ptr anyNode)
{
	std::set<Node::Ptr> visited = {};
	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
		visited.insert(current);
		for (auto&& output : current->getOutputs()) {
			if (!visited.contains(output)) {
				dfsRec(output);
			}
		}
		for (auto&& input : current->getInputs()) {
			if (!visited.contains(input)) {
				dfsRec(input);
			}
		}
	};
	dfsRec(anyNode);
	return visited;
}

static void removeNotExecutingNodes(std::set<Node::Ptr>& graph)
{
	std::set<Node::Ptr> not_visited = graph;
	std::function<void(Node::Ptr)> rmBranch = [&](Node::Ptr current) {
		for (auto&& output : current->getOutputs()) {
			rmBranch(output);
		}
		RGL_DEBUG("Removing node from execution: {}", *current);
		graph.erase(current);
		not_visited.erase(current);
	};

	while (!not_visited.empty()) {
		auto current = *not_visited.begin();
		if (!current->isActive()) {
			rmBranch(current);
		} else {
			not_visited.erase(current);
		}
	}
}

static std::vector<Node::Ptr> findTopologicalOrder(std::set<Node::Ptr> nodes, bool only_active_nodes = true)
{
	std::vector<Node::Ptr> reverseOrder {};
	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
		nodes.erase(current);
		if (only_active_nodes && !current->isActive()) {
			return;
		}
		for (auto&& output : current->getOutputs()) {
			if (nodes.contains(output)) {
				dfsRec(output);
			}
		}
		reverseOrder.push_back(current);
	};
	while (!nodes.empty()) {
		dfsRec(*nodes.begin());
	}
	return {reverseOrder.rbegin(), reverseOrder.rend()};
}

void runGraph(Node::Ptr userNode)
{
	std::set<Node::Ptr> graph = findConnectedNodes(userNode);
	removeNotExecutingNodes(graph);

	std::set<rgl_field_t> fields;
	for (auto&& formatNode : Node::filter<FormatNode>(graph)) {
		for (auto&& field : formatNode->getFieldList()) {
			if (!isDummy(field)) {
				fields.insert(field);
			}
		}
	}

	fields.insert(XYZ_F32);

	if (!Node::filter<CompactNode>(graph).empty()) {
		fields.insert(IS_HIT_I32);
	}

	RaytraceNode::Ptr rt = Node::getExactlyOne<RaytraceNode>(graph);
	rt->setFields(fields);

	std::vector<Node::Ptr> topologicalOrder = findTopologicalOrder(graph);

	for (auto&& current : topologicalOrder) {
		RGL_TRACE("Validating node: {}", *current);
		current->validate();
	}

	for (auto&& node : topologicalOrder) {
		RGL_TRACE("Scheduling node: {}", *node);
		node->schedule(nullptr);
	}
}

void destroyGraph(Node::Ptr userNode)
{
	std::set<Node::Ptr> graph = findConnectedNodes(userNode);

	while (!graph.empty()) {
		Node::Ptr node = *graph.begin();
		RGL_DEBUG("Destroying node {}", (void*) node.get());
		graph.erase(node);
		node->inputs.clear();
		node->outputs.clear();
		Node::release(node.get());
	}
}
