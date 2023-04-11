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
#include <thread>

#include <CudaStream.hpp>
#include <graph/Node.hpp>
#include <graph/NodesCore.hpp>

/**
 * Structure storing context for running a graph.
 * This is a 'volatile' struct - changing graph's structure destroys this context.
 */
struct GraphRunCtx
{
	static std::shared_ptr<GraphRunCtx> createAndAttach(std::shared_ptr<Node> node);

	/**
	 * Executes graph bound with this GraphRunCtx.
	 */
	void executeAsync();

	/**
	* Resets graphRunCtx for all nodes currently bound with this GraphRunCtx, consequently destroying this GraphRunCtx.
	*/
	void detachAndDestroy();

	/**
	 * Waits until this GraphRunCtx
	 * - finishes execution
	 * - joins its thread
	 * - synchronizes graph stream (all pending GPU operations)
	 */
	void synchronize();

	/**
	 * Ensures that no GraphRunCtx is running.
	 */
	static void synchronizeAll();

	/**
	 * Waits until given node finishes its CPU execution.
	 * The node may still have pending GPU operations.
	 */
	void synchronizeNodeCPU(Node::Ptr nodeToSynchronize);


	CudaStream::Ptr getStream() const { return stream; }
	const std::set<std::shared_ptr<Node>>& getNodes() const { return nodes; }

	virtual ~GraphRunCtx();
private:
	GraphRunCtx() : stream(CudaStream::create(cudaStreamNonBlocking)) {}

	static std::vector<std::shared_ptr<Node>> findExecutionOrder(std::set<std::shared_ptr<Node>> nodes);

	void executeThreadMain();

private:
	CudaStream::Ptr stream;
	std::optional<std::thread> maybeThread;
	std::set<Node::Ptr> nodes;
	std::vector<Node::Ptr> executionOrder;

	// Used to synchronize all existing instances (e.g. to safely access Scene).
	static std::list<std::weak_ptr<GraphRunCtx>> instances;

private: // Communication between client's thread and graph thread
	struct NodeExecStatus
	{
		// If true, then execution has succeeded or an exception has been thrown.
		std::atomic<bool> executed {false};

		// exceptionPtr may be read by client's thread only after it acquire-read true `completed`
		// exceptionPtr may be written by graph thread only before it release-stores true `completed`
		std::exception_ptr exceptionPtr {nullptr};
	};

	std::unordered_map<Node::Ptr, NodeExecStatus> executionStatus;
	std::atomic<bool> execThreadCanStart;
};
