#include <pipeline/Node.hpp>
#include <list>
#include <queue>


// template<bool bidirectional>
// std::set<Node::Ptr> dfs(Node::Ptr start, std::function<void(Node::Ptr)> onEntry=[](Node::Ptr){}, std::function<void()> onLeave=[](){})
// {
// 	std::set<Node::Ptr> visited = {};
//
// 	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
// 		std::invoke(onEntry);
// 		visited.insert(current);
//
// 		for (auto&& output : current->outputs) {
// 			if (!visited.contains(output)) {
// 				dfsRec(output);
// 			}
// 		}
//
// 		if (bidirectional) {
// 			for (auto&& input : current->inputs) {
// 				if (!visited.contains(input)) {
// 					dfsRec(input);
// 				}
// 			}
// 		}
// 		std::invoke(onLeave);
// 	};
//
// 	dfsRec(start);
// 	return visited;
// }

std::set<Node::Ptr> findConnectedNodes(Node::Ptr anyNode)
{
	std::set<Node::Ptr> visited = {};
	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
		visited.insert(current);
		for (auto&& output : current->outputs) {
			if (!visited.contains(output)) {
				dfsRec(output);
			}
		}
		for (auto&& input : current->inputs) {
			if (!visited.contains(input)) {
				dfsRec(input);
			}
		}
	};
	dfsRec(anyNode);
	return visited;
}

std::vector<Node::Ptr> findTopologicalOrder(std::set<Node::Ptr> nodes)
{
	std::vector<Node::Ptr> reverseOrder {};
	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
		nodes.erase(current);
		for (auto&& output : current->outputs) {
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


void runPipeline(Node::Ptr userNode)
{
	std::set<Node::Ptr> graph = findConnectedNodes(userNode);

	std::set<rgl_field_t> fields;
	for (auto&& formatNode : Node::filter<FormatNode>(graph)) {
		for (auto&& field : formatNode->getFieldList()) {
			fields.insert(field);
		}
	}

	if (!Node::filter<WritePCDFileNode>(graph).empty()) {
		// Currently WritePCDFileNode works only with XYZP format
		fields.insert(RGL_FIELD_XYZP_F32);
	}

	RaytraceNode::Ptr rt = Node::getExactlyOne<RaytraceNode>(graph);
	rt->setFields(fields);

	for (auto&& current : graph) {
		RGL_TRACE("Validating node: {}", *current);
		current->validate();
	}


	std::vector<Node::Ptr> executionOrder = findTopologicalOrder(graph);

	for (auto&& node : executionOrder) {
		RGL_TRACE("Scheduling node: {}", *node);
		node->schedule(nullptr);
	}
}

void destroyPipeline(Node::Ptr userNode)
{
	std::set<Node::Ptr> graph = findConnectedNodes(userNode);

	while (!graph.empty()) {
		Node::Ptr node = *graph.begin();
		graph.erase(node);
		node->inputs.clear();
		node->outputs.clear();
		Node::release(node.get());
	}
}
