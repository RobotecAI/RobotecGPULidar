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
	 * - synchronizes graph stream
	 */
	void synchronize();


	CudaStream::Ptr getStream() const { return stream; }
	const std::set<std::shared_ptr<Node>>& getNodes() const { return nodes; }

	virtual ~GraphRunCtx();
private:
	GraphRunCtx() : stream(CudaStream::create(cudaStreamNonBlocking)) {}

	static std::vector<std::shared_ptr<Node>> findExecutionOrder(std::set<std::shared_ptr<Node>> nodes);

	void execute();

private:
	CudaStream::Ptr stream;
	std::optional<std::thread> thread;

	std::set<Node::Ptr> nodes;
	std::vector<Node::Ptr> executionOrder;

	// TODO: Safe-delete??
	std::set<rgl_field_t> fieldsToCompute;

	// TODO: Safe-delete
	static std::list<std::shared_ptr<GraphRunCtx>> instances;
};
