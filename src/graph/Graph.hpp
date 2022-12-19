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

#include <graph/Node.hpp>
#include <graph/graph.hpp>

/**
 * Graph is an optional per-graph (a set of connected nodes) structure.
 * It encapsulates all the resources to start and track graph's execution.
 * GraphCtx should be considered a heavy structure (mostly due to CUDA stream creation time),
 * therefore it is are created lazily, usually on the first graph execution.
 */
struct Graph
{
	using Ptr = std::shared_ptr<Graph>;

	/** Calls non-static run on the graph represented by the given node **/
	static void run(Node::Ptr);

	/** Calls non-static sync on the graph represented by the given node **/
	static void sync(Node::Ptr);

	/** Calls non-static destroy on the graph represented by the given node **/
	static void destroy(Node::Ptr);

	static const std::vector<Node::Ptr>& getExecOrder(Node::Ptr);
	static const std::set<Node::Ptr>& getConnectedNodes(Node::Ptr);

	void run();
	void sync();
	void destroy();

	const std::set<Node::Ptr>& getExecOrder();
	const std::vector<Node::Ptr>& getConnectedNodes();


private:
	cudaStream_t cudaStream {nullptr};
	std::set<Node::Ptr> connectedNodes;
	std::vector<Node::Ptr> execOrder;

private:
	static Graph::Ptr getOrCreate(Node::Ptr node)
	{
		if (node->graph.has_value()) {
			return node->graph.value();
		}
		auto graphCtx = std::shared_ptr<Graph>(new Graph(node), [](Graph* ctx){ctx->Graph();});
		for (auto&& node : graphCtx->connectedNodes) {
			node->graph = graphCtx;
		}
		return graphCtx;
	}

	Graph(Node::Ptr node)
	{
		CHECK_CUDA(cudaStreamCreate(&cudaStream));
		connectedNodes = findConnectedNodes(node);
		execOrder = findExecutionOrder(connectedNodes);

	}

	~Graph()
	{
		if (cudaStream != nullptr) {
			CHECK_CUDA_NO_THROW(cudaStreamDestroy(cudaStream));
			cudaStream = nullptr;
		}
	}
};

