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

#include <graph/Node.hpp>
#include <graph/graph.hpp>

API_OBJECT_INSTANCE(Node);

void Node::addChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to set an empty child for {}", getName());
		throw InvalidPipeline(msg);
	}
	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	if (childIt != this->outputs.end()) {
		auto msg = fmt::format("attempted to add child {} to parent {} twice", child->getName(), getName());
		throw InvalidPipeline(msg);
	}

	this->clearExecutionContext();
	child->clearExecutionContext();

	this->outputs.push_back(child);
	child->inputs.push_back(shared_from_this());
}

void Node::removeChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to remove an empty child for {}", getName());
		throw InvalidPipeline(msg); // TODO(prybicki): should be unrecoverable
	}

	// Check, if it's our child
	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	bool itIsOurChild = childIt != this->outputs.end();
	if (!itIsOurChild) {
		auto msg = fmt::format("attempted to remove child {} from {},"
		                       "but it was not found", child->getName(), getName());
		throw InvalidPipeline(msg); // TODO(prybicki): should be unrecoverable
	}

	// Check, if we're the parent of the child
	auto thisIt = std::find(child->inputs.begin(), child->inputs.end(), shared_from_this());
	bool weAreParentOfTheChild = thisIt != child->inputs.end();
	if (!weAreParentOfTheChild) {
		auto msg = fmt::format("attempted to remove parent {} from {},"
		                       "but it was not found", getName(), child->getName());
		throw InvalidPipeline(msg); // TODO(prybicki): should be unrecoverable
	}

	bool isExecCtxShared = execCtx.has_value() == child->execCtx.has_value();
	if (!isExecCtxShared) {
		// TODO(prybicki): All the ifs here should throw a stronger (unrecoverable) exception if an inconsistent state is found
		auto msg = fmt::format("inconsistent execCtx state between parent {} and child {} ({}, {})",
		                       getName(), child->getName(), execCtx.has_value(), child->execCtx.has_value());
		throw InvalidPipeline(msg); // TODO(prybicki): should be unrecoverable
	}

	this->outputs.erase(childIt);  // Remove child from our children
	child->inputs.erase(thisIt);  // Remove us as a parent of that child

	this->clearExecutionContext();
	child->clearExecutionContext();
}

void Node::clearExecutionContext()
{
	if (!execCtx.has_value()) {
		return;  // Empty execCtx implies that all connected nodes
	}
	// Note: recycling CUDA streams is possible here
	for (auto&& node : findConnectedNodes(shared_from_this())) {
		// TODO(prybicki): at the review, make sure the implementation is complete
		if (!execCtx.has_value()) {
			// This is a bit defensive, but may detect implementation bugs
			auto msg = fmt::format("attempted to clear execCtx of {}, which is already empty", node->getName());
			throw InvalidPipeline(msg); // TODO(prybicki): should be unrecoverable
		}
		execCtx.reset();
	}
}
