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
#include <graph/Graph.hpp>

API_OBJECT_INSTANCE(Node);

Node::Node()
{
    execCompleted = CudaEvent::create();
}

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
	Graph::destroy(shared_from_this(), true);
	Graph::destroy(child, true);

	// Remove links
	this->outputs.push_back(child);
	child->inputs.push_back(shared_from_this());

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
		                       "but it was not found", child->getName(), getName());
		throw InvalidPipeline(msg);
	}
	
	auto thisIt = std::find(child->inputs.begin(), child->inputs.end(), shared_from_this());
	if (thisIt == child->inputs.end()) {
		auto msg = fmt::format("attempted to remove parent {} from {},"
		                       "but it was not found", getName(), child->getName());
		throw InvalidPipeline(msg);
	}

	// This might be a subject for future optimization.
	// Invalidate graphCtx for all connected nodes of child and parent
	// Destroy graph (by invariant, shared by child and parent)
	Graph::destroy(shared_from_this(), true);

	// Remove links
	this->outputs.erase(childIt);
	child->inputs.erase(thisIt);

	this->dirty = true;
	child->dirty = true;
}

std::shared_ptr<Graph> Node::getGraph()
{
	if (auto outGraph = graph.lock()) {
		return outGraph;
	}
	return Graph::create(shared_from_this());
}

void Node::setGraph(std::shared_ptr<Graph> newGraph)
{
	arrayMgr.setStream(newGraph->getStream());
	this->graph = newGraph;
}

void Node::validate()
{
	if (isValid()) {
		return;
	}
	if (!hasGraph()) {
		auto msg = fmt::format("{}: attempted to call validate() despite !hasGraph()", getName());
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
	this->enqueueExecImpl(getGraph()->getStream()->get());

	CHECK_CUDA(cudaEventRecord(execCompleted->get(), getGraph()->getStream()->get()));
}

