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

std::list<std::weak_ptr<GraphRunCtx>> GraphRunCtx::instances;

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

	GraphRunCtx::instances.push_back(graphRunCtx);

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

	// Clear execution states
	executionStatus.clear();
	for (auto&& node : executionOrder) {
		executionStatus.try_emplace(node);
	}
	execThreadCanStart.store(false, std::memory_order_relaxed);

	// TODO: this also applies to validation and executionStatus clearing
	// In other parts of the code we rely on the following logic
	// IF graph thread is working THEN this->thread.has_value()
	// However, there is a very tight gap, where this is not true,
	// because the graph thread could start before maybeThread is assigned.
	// Therefore, the graph thread waits for execThreadCanStart
	// which is set to true after client maybeThread has been assigned.
	maybeThread = std::thread(&GraphRunCtx::executeThreadMain, this);
	execThreadCanStart.store(true, std::memory_order::release);
}


void GraphRunCtx::executeThreadMain() try
{
	// Wait for trigger from the client's thread
	while (!execThreadCanStart.load(std::memory_order::acquire))
		;

	for (auto&& node : executionOrder) {
		// SPDLOG is thread safe, so we can log here.
		RGL_DEBUG("Enqueueing node: {}", *node);
		node->enqueueExec();
		executionStatus.at(node).executed.store(true, std::memory_order::release);
	}
	RGL_DEBUG("Node enqueueing done");  // This also logs the time diff for the last one

	CHECK_CUDA(cudaStreamSynchronize(stream->getHandle()));
}
catch (...)
{
	// Exception most likely happened in a Node, but might have happened around executionOrder loop.
	// We still need to communicate that nodes 'executed' (even though some may not have a chance to start).
	// If we didn't, we could hang client's thread in synchronizeNodeCPU() waiting for a Node that will never run.
	for (auto&& [node, state] : executionStatus) {
		if (state.executed.load(std::memory_order::relaxed)) {
			continue;
		}
		state.exceptionPtr = std::current_exception();
		state.executed.store(true, std::memory_order::release);
	}
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
	cudaError_t status = cudaStreamQuery(stream->getHandle());
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
	this->synchronize();
	for (auto&& node : nodes) {
		node->setGraphRunCtx(std::nullopt);
	}
	// After this loop, we should have removed all shared_ptrs to GraphRunCtx, so it will be destroyed.
}

void GraphRunCtx::synchronize()
{
	if (!maybeThread.has_value()) {
		return; // Already synchronized or never run.
	}
	// This order must be preserved.
	for (auto&& node : executionOrder) {
		synchronizeNodeCPU(node);
	}
	CHECK_CUDA(cudaStreamSynchronize(stream->getHandle()));
	maybeThread->join();
	maybeThread.reset();
}

void GraphRunCtx::synchronizeNodeCPU(Node::Ptr nodeToSynchronize)
{
	if (!maybeThread.has_value()) {
		return; // Already synchronized or never run.
	}
	// Wait until node is executed
	// This is call executed in client's thread, which is often engine's main thread.
	// Therefore, we choose busy wait, since putting it to sleep & waking up would add additional latency.
	while (!executionStatus.at(nodeToSynchronize).executed.load(std::memory_order_acquire))
		;
	// Rethrow exception, if any
	if (auto ex = executionStatus.at(nodeToSynchronize).exceptionPtr) {
		// Do not clear exception ptr yet, it could give false image that the Node is OK.
		std::rethrow_exception(ex);
	}
}

void GraphRunCtx::synchronizeAll()
{
	auto it = instances.begin();
	for (; it != instances.end(); ++it)
	{
		auto&& ctxWeakPtr = *it;
		auto ctxSharedPtr = ctxWeakPtr.lock();
		if (ctxSharedPtr == nullptr) {
			it = instances.erase(it);
			continue;
		}
		ctxSharedPtr->synchronize();
	}
}