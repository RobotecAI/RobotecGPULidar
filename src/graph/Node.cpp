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
#include <graph/NodesCore.hpp>
#include <graph/GraphRunCtx.hpp>

API_OBJECT_INSTANCE(Node);

Node::Node() { execCompleted = CudaEvent::create(); }

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

	// This might be a subject for future optimization.
	// Invalidate graphCtx for all connected nodes of child and parent
	if (this->hasGraphRunCtx()) {
		this->getGraphRunCtx()->detachAndDestroy();
	}
	if (child->hasGraphRunCtx()) {
		child->getGraphRunCtx()->detachAndDestroy();
	}

	// Remove links
	this->outputs.push_back(child);
	child->inputs.push_back(shared_from_this());

	// Sort outputs by priority
	std::stable_sort(outputs.begin(), outputs.end(),
	                 [](Node::Ptr lhs, Node::Ptr rhs) { return lhs->priority > rhs->priority; });

	auto maxChildPriority = outputs.front()->priority;
	if (maxChildPriority > this->priority) {
		this->setPriority(maxChildPriority);
	}

	this->dirty = true;
	child->dirty = true;
}

void Node::removeChild(Node::Ptr child)
{
	if (child == nullptr) {
		auto msg = fmt::format("attempted to remove an empty child for {}", getName());
		throw InvalidPipeline(msg);
	}

	auto childIt = std::find(this->outputs.begin(), this->outputs.end(), child);
	if (childIt == this->outputs.end()) {
		auto msg = fmt::format("attempted to remove child {} from {},"
		                       "but it was not found",
		                       child->getName(), getName());
		throw InvalidPipeline(msg);
	}

	auto thisIt = std::find(child->inputs.begin(), child->inputs.end(), shared_from_this());
	if (thisIt == child->inputs.end()) {
		auto msg = fmt::format("attempted to remove parent {} from {},"
		                       "but it was not found",
		                       getName(), child->getName());
		throw InvalidPipeline(msg);
	}

	// This might be a subject for future optimization.
	// Remove graphCtx for all connected nodes of child and parent
	// Destroy graph (by invariant, shared by child and parent)
	if (this->hasGraphRunCtx()) {
		this->getGraphRunCtx()->detachAndDestroy();
	}

	// Remove links
	this->outputs.erase(childIt);
	child->inputs.erase(thisIt);

	this->dirty = true;
	child->dirty = true;
}

void Node::setGraphRunCtx(std::optional<std::shared_ptr<GraphRunCtx>> graph)
{
	arrayMgr.setStream(graph.has_value() ? graph.value()->getStream() : CudaStream::getNullStream());
	this->graphRunCtx = graph;
	this->dirty = true;
}

void Node::validate()
{
	if (isValid()) {
		return;
	}
	if (!hasGraphRunCtx()) {
		auto msg = fmt::format("{}: attempted to call validate() despite !hasGraphRunCtx()", getName());
		throw std::logic_error(msg);
	}
	this->validateImpl();
	dirty = false;
}

void Node::enqueueExec()
{
	if (!isValid()) {
		auto msg = fmt::format("{}: attempted to call enqueueExec() despite !isValid()", getName());
		throw std::logic_error(msg);
	}
	this->enqueueExecImpl();
	CHECK_CUDA(cudaEventRecord(execCompleted->getHandle(), getGraphRunCtx()->getStream()->getHandle()));
}

std::set<Node::Ptr> Node::getConnectedComponentNodes()
{
	if (hasGraphRunCtx()) {
		return getGraphRunCtx()->getNodes();
	}
	std::set<Ptr> visited = {};
	std::function<void(Ptr)> dfsRec = [&](Ptr current) {
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
	dfsRec(shared_from_this());
	return visited;
}

std::set<Node::Ptr> Node::disconnectConnectedNodes()
{
	auto nodes = getConnectedComponentNodes();
	for (auto&& node : getConnectedComponentNodes()) {
		node->inputs.clear();
		node->outputs.clear();
		node->setGraphRunCtx(std::nullopt);
	}
	return nodes;
}

void Node::synchronize() const
{
	if (!hasGraphRunCtx()) {
		return; // Nothing to synchronize ¯\_(ツ)_/¯
	}
	// Ensure CPU execution is finished; this guarantees that execCompleted event has been recorded
	graphRunCtx.value()->synchronizeNodeCPU(shared_from_this());
	// Wait for all GPU operations requested by this node.
	CHECK_CUDA(cudaEventSynchronize(execCompleted->getHandle()));
}

void Node::waitForResults()
{
	// TODO: merge with synchronize?
	// ...
	if (!isValid()) {
		auto msg = fmt::format("Cannot get results from {}; it hasn't been run yet, or the run has failed", getName());
		throw InvalidPipeline(msg);
	}
	synchronize();
}

cudaStream_t Node::getStreamHandle() { return getGraphRunCtx()->getStream()->getHandle(); }

void Node::setPriority(int32_t requestedPriority)
{
	if (requestedPriority == priority) {
		return;
	}
	// It is illegal to set node's priority to a value lower than max priority value of its outputs.
	// This keeps the invariant of node's priority being always >= than priority of its children.
	if (!outputs.empty() && outputs.front()->priority > requestedPriority) {
		auto msg = fmt::format("cannot set priority of node {} to {}, because it's children {} has higher priority {}",
		                       getName(), requestedPriority, outputs.front()->getName(), outputs.front()->priority);
		throw InvalidPipeline(msg);
	}

	this->priority = requestedPriority;
	// Update parent nodes.
	for (auto&& input : inputs) {
		// Resort parent's list of children, because order may have changed.
		std::stable_sort(input->outputs.begin(), input->outputs.end(),
		                 [](Node::Ptr lhs, Node::Ptr rhs) { return lhs->priority > rhs->priority; });
		// Increase parents' priority
		if (input->priority < requestedPriority) {
			input->setPriority(requestedPriority);
		}
	}
	if (hasGraphRunCtx()) {
		// Synchronized with Graph thread on API level
		graphRunCtx.value()->executionOrder.clear();
	}
}