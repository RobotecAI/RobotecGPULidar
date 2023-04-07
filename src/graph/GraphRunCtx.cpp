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

std::shared_ptr<GraphRunCtx> GraphRunCtx::createAndAttach(std::shared_ptr<Node> node)
{
	auto graphRunCtx = std::shared_ptr<GraphRunCtx>(new GraphRunCtx());

	graphRunCtx->nodes = node->getConnectedNodes();
	graphRunCtx->executionOrder = GraphRunCtx::findExecutionOrder(graphRunCtx->nodes);

	for (auto&& currentNode : graphRunCtx->nodes) {
		if (currentNode->hasGraphRunCtx()) {
			auto msg = fmt::format("attempted to replace existing graphRunCtx in node {} when creating for {}", currentNode->getName(), node->getName());
			throw std::logic_error(msg);
		}
		currentNode->setGraphRunCtx(graphRunCtx);
	}

	return graphRunCtx;
}

void GraphRunCtx::executeAsync()
{
	synchronize(); // Wait until previous execution is completed

	// Perform validation in client's thread, this makes error reporting easier.
	for (auto&& current : executionOrder) {
		RGL_DEBUG("Validating node: {}", *current);
		current->validate();
	}
	RGL_DEBUG("Node validation completed");  // This also logs the time diff for the last one.

	// In other parts of the code we rely on the following logic
	// IF thread is executing THEN this->thread.has_value()
	// However, there is a very tight gap, where this is not true,
	// because the thread could start before maybeThread is assigned.
	// Therefore, the thread wait for execThreadCanStart
	// which is set to true after client maybeThread has been assigned.
	execThreadCanStart = false;
	maybeThread = std::thread(&GraphRunCtx::executeThreadMain, this);
	execThreadCanStart = true;
}


void GraphRunCtx::executeThreadMain()
{
	while (!execThreadCanStart)
		;
	for (auto&& node : executionOrder) {
		RGL_DEBUG("Enqueueing node: {}", *node);
		node->enqueueExec();
	}
	RGL_DEBUG("Node enqueueing done");  // This also logs the time diff for the last one

	CHECK_CUDA(cudaStreamSynchronize(stream->get()));
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

GraphRunCtx::~GraphRunCtx()
{
	// If GraphRunCtx is destroyed, we expect that thread was joined and stream was synced.

	// Log error if stream has pending work.
	cudaError_t status = cudaStreamQuery(stream->get());
	if (status == cudaErrorNotReady) {
		RGL_WARN("~GraphRunCtx(): stream has pending work!");
		status = cudaSuccess; // Ignore further checks.
	}
	CHECK_CUDA_NO_THROW(status);

	if (maybeThread.has_value()) {
		RGL_WARN("~GraphRunCtx(): maybeThread has value!");
	}
}

void GraphRunCtx::detachAndDestroy()
{
	synchronize();
	for (auto&& node : nodes) {
		node->setGraphRunCtx(std::nullopt);
	}
	// After this loop, we should have removed all shared_ptrs to GraphRunCtx, so it will be destroyed.
}

void GraphRunCtx::synchronize()
{
	// This order must be preserved.
	CHECK_CUDA(cudaStreamSynchronize(stream->get()));
	if (maybeThread.has_value()) {
		maybeThread->join();
		maybeThread.reset();
	}
}
