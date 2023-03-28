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

struct Graph;

struct Node : APIObject<Node>, std::enable_shared_from_this<Node>
{
	using Ptr = std::shared_ptr<Node>;

	virtual ~Node() override = default;

	void addChild(Node::Ptr child);
	void removeChild(Node::Ptr child);

	/**
	 * Notifies node about changes in their input nodes.
	 * The node may want to react by updating their cached input handle.
	 */
	void onInputChange();

	/**
	 * Prepare node computation and insert it into the given stream.
	 * Note: This method may cause stream synchronization!
	 * This function will throw if canExecute() returns false.
	 * @param stream Stream to perform computations in.
	 */
	void execute();

	std::shared_ptr<Graph> getGraph();

	bool hasGraph() { return graph.lock() != nullptr; }
	bool canExecute() { return inputOK; }
	bool isLastExecOk() { return lastExecOK; }
	std::string getName() const { return name(typeid(*this)); }

	/* Nodes may optionally override this function to provide debug info about their arguments */
	virtual std::string getArgsString() const { return {}; }

	const std::vector<Node::Ptr>& getInputs() const { return inputs; }
	const std::vector<Node::Ptr>& getOutputs() const { return outputs; }

protected:
	virtual void onInputChangeImpl() = 0;
	virtual void executeImpl(cudaStream_t stream) = 0;

	template<template<typename, typename...> typename Container, typename...CArgs>
	static std::string getNodeTypeNames(const Container<Node::Ptr, CArgs...>& nodes, std::string_view separator=", ")
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

	template<typename T, template<typename, typename...> typename Container, typename...CArgs>
	static std::vector<typename T::Ptr> filter(const Container<Node::Ptr, CArgs...>& nodes)
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

	template<typename T, template<typename, typename...> typename Container, typename...CArgs>
	static typename T::Ptr getExactlyOne(const Container<Node::Ptr, CArgs...>& nodes)
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
	Node() = default;

protected:
	std::vector<Node::Ptr> inputs {};
	std::vector<Node::Ptr> outputs {};

	std::weak_ptr<Graph> graph;
	bool inputOK { false };
	bool lastExecOK { false };

	friend struct Graph;
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

		return fmt::format_to(ctx.out(), fmt::runtime("{}{{in=[{}], out=[{}]}}({})"),
		                      node.getName(),
		                      Node::getNodeTypeNames(node.inputs),
		                      Node::getNodeTypeNames(node.outputs),
		                      node.getArgsString());
	}
};
#endif // __CUDACC__
