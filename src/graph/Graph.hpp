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

#pragma once

#include <list>
#include <set>
#include <vector>

#include <graph/Node.hpp>
#include <CudaStream.hpp>

struct Graph
{
	// Public - tape uses it
	static std::set<std::shared_ptr<Node>> findConnectedNodes(std::shared_ptr<Node> anyNode);

	static std::shared_ptr<Graph> create(std::shared_ptr<Node> node);
	static void destroy(std::shared_ptr<Node> anyNode, bool preserveNodes);

	void run();
	const std::set<std::shared_ptr<Node>>& getNodes() const { return nodes; }
	const std::vector<std::shared_ptr<Node>>& getExecutionOrder();

	virtual ~Graph();
private:
	Graph() { stream = std::make_shared<CudaStream>(); }

	static std::vector<std::shared_ptr<Node>> findExecutionOrder(std::set<std::shared_ptr<Node>> nodes);

private:
	std::shared_ptr<CudaStream> stream;
	std::set<std::shared_ptr<Node>> nodes;

	// Execution order may change due to e.g., node (de)activation
	std::optional<std::vector<std::shared_ptr<Node>>> executionOrder;

	static std::list<std::shared_ptr<Graph>> instances;

	friend struct Node;
};
