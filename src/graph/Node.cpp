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

	Graph::destroy(shared_from_this(), true);
	Graph::destroy(child, true);

	this->outputs.push_back(child);
	child->inputs.push_back(shared_from_this());
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

	Graph::destroy(shared_from_this(), true);

	// Remove child from our children
	this->outputs.erase(childIt);

	// Remove us as a parent of that child
	child->inputs.erase(thisIt);
}

std::shared_ptr<Graph> Node::getGraph()
{
	if (auto outGraph = graph.lock()) {
		return outGraph;
	}
	return Graph::create(shared_from_this());
}

void Node::execute()
{
	if (!inputOK) {
		onInputChange();
	}
	executeImpl(nullptr);
	// TODO: insert CUDA event to the stream
}

void Node::onInputChange()
{
	try
	{
		onInputChangeImpl();
		inputOK = true;
	}
	catch (...)
	{
		inputOK = false;
		throw;
	}
}