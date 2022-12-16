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

#include <set>
#include <vector>
#include <graph/graph.hpp>
#include <graph/NodesCore.hpp>
#include <RGLFields.hpp>

std::set<Node::Ptr> findConnectedNodes(Node::Ptr anyNode)
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

static std::vector<Node::Ptr> findExecutionOrder(std::set<Node::Ptr> nodes)
{
	std::vector<Node::Ptr> reverseOrder {};
	std::function<void(Node::Ptr)> rmBranch = [&](Node::Ptr current) {
		for (auto&& output : current->getOutputs()) {
			rmBranch(output);
		}
		RGL_DEBUG("Removing node from execution: {}", *current);
		nodes.erase(current);
	};
	std::function<void(Node::Ptr)> dfsRec = [&](Node::Ptr current) {
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

void runGraph(Node::Ptr userNode)
{
	std::vector<Node::Ptr> nodesInExecOrder = findExecutionOrder(findConnectedNodes(userNode));

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
