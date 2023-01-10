// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <graph/Graph.hpp>
#include <graph/NodesCore.hpp>

std::list<std::shared_ptr<Graph>> Graph::instances;

std::set<std::shared_ptr<Node>> Graph::findConnectedNodes(std::shared_ptr<Node> anyNode)
{
	std::set<std::shared_ptr<Node>> visited = {};
	std::function<void(std::shared_ptr<Node>)> dfsRec = [&](std::shared_ptr<Node> current) {
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

std::shared_ptr<Graph> Graph::create(std::shared_ptr<Node> node)
{
	auto graph = std::shared_ptr<Graph>(new Graph());

	graph->nodes = Graph::findConnectedNodes(node);

	for (auto&& currentNode : graph->nodes) {
		if (currentNode->hasGraph()) {
			auto msg = fmt::format("attempted to replace existing graph in node {} when creating for {}", currentNode->getName(), node->getName());
			throw std::logic_error(msg);
		}
		currentNode->graph = graph;
	}

	instances.push_back(graph);

	return graph;
}

void Graph::run()
{
	const auto& nodesInExecOrder = getExecutionOrder();

	RGL_DEBUG("Running graph with {} nodes", nodesInExecOrder.size());

	std::set<rgl_field_t> fieldsToCompute;
	for (auto&& node : nodesInExecOrder) {
		if (auto pointNode = std::dynamic_pointer_cast<IPointsNode>(node)) {
			for (auto&& field : pointNode->getRequiredFieldList()) {
				if (!isDummy(field)) {
					fieldsToCompute.insert(field);
				}
			}
		}
	}

	fieldsToCompute.insert(XYZ_F32);

	if (!Node::filter<CompactPointsNode>(nodesInExecOrder).empty()) {
		fieldsToCompute.insert(IS_HIT_I32);
	}

	RaytraceNode::Ptr rt = Node::getExactlyOne<RaytraceNode>(nodesInExecOrder);
	rt->setFields(fieldsToCompute);

	for (auto&& current : nodesInExecOrder) {
		RGL_DEBUG("Validating node: {}", *current);
		current->validate();
	}
	RGL_DEBUG("Node validation completed");  // This also logs the time diff for the last one.

	for (auto&& node : nodesInExecOrder) {
		RGL_DEBUG("Scheduling node: {}", *node);
		node->schedule(nullptr);
	}
	RGL_DEBUG("Node scheduling done");  // This also logs the time diff for the last one
}

void Graph::destroy(std::shared_ptr<Node> anyNode, bool preserveNodes)
{
	if (!preserveNodes) {
		std::set<std::shared_ptr<Node>> graphNodes = anyNode->hasGraph()
		                                           ? anyNode->getGraph()->getNodes()
												   : Graph::findConnectedNodes(anyNode); 
		while (!graphNodes.empty()) {
			std::shared_ptr<Node> node = *graphNodes.begin();
			RGL_DEBUG("Destroying node {}", (void*) node.get());
			graphNodes.erase(node);
			node->inputs.clear();
			node->outputs.clear();
			Node::release(node.get());
		}
	}

	if (!anyNode->hasGraph()) {
		return;
	}

	auto graphIt = std::find(instances.begin(), instances.end(), anyNode->getGraph());
	if (graphIt == instances.end()) {
		auto msg = fmt::format("attempted to remove a graph not in Graph::instances");
		throw std::logic_error(msg);
	}
	instances.erase(graphIt);
}

const std::vector<std::shared_ptr<Node>>& Graph::getExecutionOrder()
{
	if (!executionOrder.has_value()) {
		executionOrder = findExecutionOrder(nodes);
	}
	return executionOrder.value();
}

std::vector<std::shared_ptr<Node>> Graph::findExecutionOrder(std::set<std::shared_ptr<Node>> nodes)
{
	std::vector<std::shared_ptr<Node>> reverseOrder {};
	std::function<void(std::shared_ptr<Node>)> rmBranch = [&](std::shared_ptr<Node> current) {
		for (auto&& output : current->getOutputs()) {
			rmBranch(output);
		}
		RGL_DEBUG("Removing node from execution: {}", *current);
		nodes.erase(current);
	};
	std::function<void(std::shared_ptr<Node>)> dfsRec = [&](std::shared_ptr<Node> current) {
		if (!current->isActive()) {
			rmBranch(current);
			return;
		}
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

Graph::~Graph()
{
	stream.reset();
}

