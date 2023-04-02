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

#include <graph/GraphRunCtx.hpp>
#include <graph/NodesCore.hpp>
#include <graph/Node.hpp>

std::list<std::shared_ptr<GraphRunCtx>> GraphRunCtx::instances;

std::shared_ptr<GraphRunCtx> GraphRunCtx::create(std::shared_ptr<Node> node)
{
	auto graphRunCtx = std::shared_ptr<GraphRunCtx>(new GraphRunCtx());

	graphRunCtx->nodes = node->getConnectedNodes();
	graphRunCtx->executionOrder = GraphRunCtx::findExecutionOrder(graphRunCtx->nodes);
	graphRunCtx->fieldsToCompute = GraphRunCtx::findFieldsToCompute(graphRunCtx->nodes);

	for (auto&& currentNode : graphRunCtx->nodes) {
		if (currentNode->hasGraphRunCtx()) {
			auto msg = fmt::format("attempted to replace existing graphRunCtx in node {} when creating for {}", currentNode->getName(), node->getName());
			throw std::logic_error(msg);
		}
		currentNode->setGraphRunCtx(graphRunCtx);
	}

	instances.push_back(graphRunCtx);

	return graphRunCtx;
}

void GraphRunCtx::run()
{
	const auto& nodesInExecOrder = executionOrder;

	RGL_DEBUG("Running graph with {} nodes", nodesInExecOrder.size());

	// If Graph has RaytraceNode set fields to compute
	if (!Node::getNodesOfType<RaytraceNode>(nodesInExecOrder).empty()) {
		RaytraceNode::Ptr rt = Node::getExactlyOneNodeOfType<RaytraceNode>(nodesInExecOrder);
		rt->setFields(fieldsToCompute);
	}

	for (auto&& current : nodesInExecOrder) {
		RGL_DEBUG("Validating node: {}", *current);
		current->validate();
	}
	RGL_DEBUG("Node validation completed");  // This also logs the time diff for the last one.

	for (auto&& node : nodesInExecOrder) {
		RGL_DEBUG("Enqueueing node: {}", *node);
		node->enqueueExec();
	}
	RGL_DEBUG("Node enqueueing done");  // This also logs the time diff for the last one
}

void GraphRunCtx::destroy(std::shared_ptr<Node> anyNode, bool preserveNodes)
{
	if (!preserveNodes) {
		std::set<std::shared_ptr<Node>> graphNodes = anyNode->getConnectedNodes();
		while (!graphNodes.empty()) {
			std::shared_ptr<Node> node = *graphNodes.begin();
			RGL_DEBUG("Destroying node {}", (void*) node.get());
			graphNodes.erase(node);
			node->inputs.clear();
			node->outputs.clear();
			Node::release(node.get());
		}
	}

	if (!anyNode->hasGraphRunCtx()) {
		return;
	}

	auto graphIt = std::find(instances.begin(), instances.end(), anyNode->getGraphRunCtx());
	if (graphIt == instances.end()) {
		auto msg = fmt::format("attempted to remove a graph not in Graph::instances");
		throw std::logic_error(msg);
	}
	instances.erase(graphIt);
}

std::vector<std::shared_ptr<Node>> GraphRunCtx::findExecutionOrder(std::set<std::shared_ptr<Node>> nodes)
{
	std::vector<std::shared_ptr<Node>> reverseOrder {};
	std::function<void(std::shared_ptr<Node>)> dfsRec = [&](std::shared_ptr<Node> current) {
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

std::set<rgl_field_t> GraphRunCtx::findFieldsToCompute(std::set<std::shared_ptr<Node>> nodes)
{
	std::set<rgl_field_t> outFields;
	for (auto&& node : nodes) {
		if (auto pointNode = std::dynamic_pointer_cast<IPointsNode>(node)) {
			for (auto&& field : pointNode->getRequiredFieldList()) {
				if (!isDummy(field)) {
					outFields.insert(field);
				}
			}
		}
	}

	outFields.insert(XYZ_F32);

	if (!Node::getNodesOfType<CompactPointsNode>(nodes).empty()) {
		outFields.insert(IS_HIT_I32);
	}

	return outFields;
}

GraphRunCtx::~GraphRunCtx()
{
	stream.reset();
}
