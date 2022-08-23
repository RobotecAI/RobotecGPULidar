#include <set>
#include <vector>
#include <pipeline/pipeline.hpp>
#include <pipeline/Nodes.hpp>
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

static std::vector<Node::Ptr> findTopologicalOrder(std::set<Node::Ptr> nodes)
{
	std::vector<Node::Ptr> reverseOrder {};
	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
		nodes.erase(current);
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

void runPipeline(Node::Ptr userNode)
{
	std::set<Node::Ptr> graph = findConnectedNodes(userNode);

	std::set<rgl_field_t> fields;
	for (auto&& formatNode : Node::filter<FormatNode>(graph)) {
		for (auto&& field : formatNode->getFieldList()) {
			if (!isDummy(field)) {
				fields.insert(field);
			}
		}
	}

	fields.insert(RGL_FIELD_XYZ_F32);

	if (!Node::filter<CompactNode>(graph).empty()) {
		fields.insert(RGL_FIELD_IS_HIT_I32);
	}

	RaytraceNode::Ptr rt = Node::getExactlyOne<RaytraceNode>(graph);
	rt->setFields(fields);

	std::vector<Node::Ptr> topologicalOrder = findTopologicalOrder(graph);

	for (auto&& current : topologicalOrder) {
		RGL_TRACE("Validating node: {}", *current);
		current->validate(nullptr);
	}

	for (auto&& node : topologicalOrder) {
		RGL_TRACE("Scheduling node: {}", *node);
		node->schedule(nullptr);
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
		if (auto pclNode = std::dynamic_pointer_cast<IPointCloudNode>(node)) {
			RGL_TRACE("PCL after {}: {}", node->getName(), pclNode->getWidth());
		}
	}
}

void destroyPipeline(Node::Ptr userNode)
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
