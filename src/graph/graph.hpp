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

#include <list>

#include <graph/Node.hpp>

/**
 * NodeExecutionContext is a structure shared between connected nodes.
 * It encapsulates all the resources to start and track graph's execution.
 * NodeExecutionContext should be considered a heavy structure (mostly due to CUDA stream creation time),
 * therefore it is are created upon a request, usually on the first graph execution.
 */
struct NodeExecutionContext
{
	using Ptr = std::shared_ptr<NodeExecutionContext>;
	// static std::list<NodeExecutionContext::Ptr> instances;
	// static NodeExecutionContext::Ptr getOrCreate(Node::Ptr);
	const std::vector<Node::Ptr>& getExecutionOrder();

private:
	std::optional<std::vector<Node::Ptr>> executionOrder;
};

std::set<Node::Ptr> findConnectedNodes(Node::Ptr graphNode);
void runGraph(Node::Ptr graphNode);
void destroyGraph(Node::Ptr graphNode);
