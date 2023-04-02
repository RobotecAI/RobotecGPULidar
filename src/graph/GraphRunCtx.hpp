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

struct GraphRunCtx
{
	static std::shared_ptr<GraphRunCtx> create(std::shared_ptr<Node> node);
	static void destroy(std::shared_ptr<Node> anyNode, bool preserveNodes);
	static std::set<std::shared_ptr<Node>> findConnectedNodes(std::shared_ptr<Node> anyNode); // Public - tape uses it

	void run();
	CudaStream::Ptr getStream() const { return stream; }

	virtual ~GraphRunCtx();
private:
	GraphRunCtx() : stream(CudaStream::create()) {}

	const std::set<std::shared_ptr<Node>>& getNodes() const { return nodes; }

	static std::vector<std::shared_ptr<Node>> findExecutionOrder(std::set<std::shared_ptr<Node>> nodes);
	static std::set<rgl_field_t> findFieldsToCompute(std::set<std::shared_ptr<Node>> nodes);

private:
	CudaStream::Ptr stream;
	std::set<std::shared_ptr<Node>> nodes;
	std::vector<std::shared_ptr<Node>> executionOrder;
	std::set<rgl_field_t> fieldsToCompute;

	static std::list<std::shared_ptr<GraphRunCtx>> instances;
};
