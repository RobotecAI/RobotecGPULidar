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

#include <memory>
#include <vector>

#include <VArray.hpp>
#include <rgl/api/core.h>
#include <APIObject.hpp>
#include <RGLFields.hpp>

struct NodeExecutionContext;

struct Node : APIObject<Node>, std::enable_shared_from_this<Node>
{
	using Ptr = std::shared_ptr<Node>;

	Node() = default;
	~Node() override = default;

	void addChild(Node::Ptr child);
	void removeChild(Node::Ptr child);

	/**
	 * Called on every node when the computation graph changes, e.g.:
	 * - a node gets inserted or removed
	 * - node parameters are changed
	 * WARNING: validate() should not depend on parents VArray buffer sizes
	 * I.E. Operations such as resizing output buffers must be done in schedule()
	 */
	virtual void validate() = 0;

	/**
	 * Prepare node computation and insert it into the given stream.
	 * Note: This method may cause stream synchronization!
	 * @param stream Stream to perform computations in
	 */
	virtual void schedule(cudaStream_t stream) = 0;

	std::string getName() const { return name(typeid(*this)); }

	const std::vector<Node::Ptr>& getInputs() const { return inputs; }
	const std::vector<Node::Ptr>& getOutputs() const { return outputs; }

	bool isActive() const { return active; }
	void setActive(bool active) { this->active = active; }

	/** Destroys execution context of all nodes in the graph this node belongs to. **/
	void clearExecutionContext();

protected:
	template<template<typename _1, typename _2> typename Container>
	static std::string getNodeTypeNames(const Container<Node::Ptr, std::allocator<Node::Ptr>>& nodes, std::string_view separator=", ")
	{
		std::string output{};
		for (auto&& node: nodes) {
			output += node->getName();
			output += separator;
		}
		if (!output.empty()) {
			std::string_view view {output};
			view.remove_suffix(separator.size()); // Remove trailing separator
			output = std::string(view);
		}
		return output;
	}

	template<typename T, template<typename _1, typename _2> typename Container>
	static std::vector<typename T::Ptr> filter(const Container<Node::Ptr, std::allocator<Node::Ptr>>& nodes)
	{
		std::vector<typename T::Ptr> typedNodes {};
		for (auto&& node : nodes) {
			auto typedNode = std::dynamic_pointer_cast<T>(node);
			if (typedNode != nullptr) {
				typedNodes.push_back(typedNode);
			}
		}
		return typedNodes;
	}

	template<typename T, template<typename _1, typename _2> typename Container>
	static typename T::Ptr getExactlyOne(const Container<Node::Ptr, std::allocator<Node::Ptr>>& nodes)
	{
		std::vector<typename T::Ptr> typedNodes = Node::filter<T>(nodes);
		if (typedNodes.size() != 1) {
			auto msg = fmt::format("looked for {}, but found [{}]", name(typeid(T)), getNodeTypeNames(nodes));
			throw InvalidPipeline(msg);
		}
		return typedNodes[0];
	}

	template<typename T>
	typename T::Ptr getValidInput()
	{ return getValidInputFrom<T>(inputs); }

	template<typename T>
	typename T::Ptr getValidInputFrom(const std::vector<Node::Ptr>& srcs)
	{ return getExactlyOne<T>(srcs); }

protected:
	bool active {true};
	std::vector<Node::Ptr> inputs {};
	std::vector<Node::Ptr> outputs {};
	std::optional<std::shared_ptr<NodeExecutionContext>> execCtx;

	friend void runGraph(Node::Ptr);
	friend void destroyGraph(Node::Ptr);
	friend struct fmt::formatter<Node>;
};

#ifndef __CUDACC__
template<>
struct fmt::formatter<Node>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) {
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(const Node& node, FormatContext& ctx) {
		return fmt::format_to(ctx.out(), fmt::runtime("{}(in=[{}], out=[{}])"),
		                      node.getName(),
		                      Node::getNodeTypeNames(node.inputs),
		                      Node::getNodeTypeNames(node.outputs));
	}
};
#endif // __CUDACC__
