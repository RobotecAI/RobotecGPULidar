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
#include <list>

#include "CudaEvent.hpp"
#include <APIObject.hpp>
#include <RGLFields.hpp>
#include <VArray.hpp>
#include <memory/StreamBoundObjectsManager.hpp>
#include <rgl/api/core.h>

struct Graph;

/**
 * Node represents a unit of computation in RGL.
 *
 * Nodes can be joined (addChild) to form a graph.
 *
 * When Node is created, it is not ready to use, because
 * - it has no GraphRunContext assigned;
 * - is in invalid state;
 *
 * Before first run, GraphRunContext is be assigned to node by caller code.
 *
 * Nodes must create DeviceAsyncArray<T>-s only through the arrayMgr.
 * It allows to change their stream when needed.
 *
 * Before every run and result query, validate() must be called to ensure that the node isValid()
 *
 * When run (enqueueExec), Nodes are expected to enqueue its computation to a stream provided by GraphRunContext.
 *
 * Between runs, Node's connection can be changed (addChild, removeChild).
 * If Node's input changes, it becomes dirty (!isValid()).
 *
 * Between runs, Node's GraphRunContext may be also changed.
 * In such scenario, Node must adjust its internal resources to work in the new GraphRunContext (new stream).
 */
struct Node : APIObject<Node>, std::enable_shared_from_this<Node>
{
	using Ptr = std::shared_ptr<Node>;

	virtual ~Node() override { execCompleted.reset(); }

        void addChild(Node::Ptr child);

	void removeChild(Node::Ptr child);

	/**
	 * Called to set/change current GraphRunContext.
	 * Node must ensure that its future operations will be enqueued
	 * to the stream associated with given graph.
	 */
	void setGraph(std::shared_ptr<Graph> graph);

	bool hasGraph() { return graph.lock() != nullptr; }
	std::shared_ptr<Graph> getGraph();

	/**
	 * Certain operations, such as adding/removing child/parent links
	 * may cause some nodes to be in an invalid state (not ready).
	 * Node must be made ready before it is executed or queried for results.
	 * Node can assume that it has valid GraphRunContext.
	 * Node must not change its output.
	 */
	void validate();

	/**
	 * @return True, if node can be executed or queried for results.
	 */
	bool isValid() { return !dirty; }

	/**
	 * Enqueues node-specific operations to the stream pointed by GraphRunContext.
	 */
	void enqueueExec();

	const std::vector<Node::Ptr>& getInputs() const { return inputs; }
	const std::vector<Node::Ptr>& getOutputs() const { return outputs; }


public: // Debug methods

	std::string getName() const { return name(typeid(*this)); }

	/**
	 * Placeholder for derived classes to provide more log/debug/error information.
	 * @return String describing Node's current parameters.
	 */
	virtual std::string getArgsString() const { return {}; }

protected: // Member methods

	Node();

	/**
	 * Placeholder to enqueue node-specific computations in derived classes.
	 */
	virtual void enqueueExecImpl(cudaStream_t toBeRemoved=nullptr) = 0;

	/**
	 * Placeholder to perform node-specific part of validation.
	 */
	virtual void validateImpl() = 0;

	template<typename T>
	typename T::Ptr getExactlyOneInputOfType()
	{ return getExactlyOneNodeOfType<T>(inputs); }

protected: // Static methods

	template<template<typename, typename...> typename Container, typename...CArgs>
	static std::string getNamesOfNodes(const Container<Node::Ptr, CArgs...>& nodes, std::string_view separator= ", ")
	{
		std::string output{};
		for (auto&& node : nodes) {
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
	static std::vector<typename T::Ptr> getNodesOfType(const Container<Node::Ptr, CArgs...>& nodes)
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
	static typename T::Ptr getExactlyOneNodeOfType(const Container<Node::Ptr, CArgs...>& nodes)
	{
		std::vector<typename T::Ptr> typedNodes = Node::getNodesOfType<T>(nodes);
		if (typedNodes.size() != 1) {
			auto msg = fmt::format("looked for {}, but found [{}]", name(typeid(T)), getNamesOfNodes(nodes));
			throw InvalidPipeline(msg);
		}
		return typedNodes[0];
	}


protected:
	std::vector<Node::Ptr> inputs {};
	std::vector<Node::Ptr> outputs {};

	bool dirty { true };
	CudaEvent::Ptr execCompleted { nullptr };

	std::weak_ptr<Graph> graph; // Pointer may be destroyed e.g. on addChild
	StreamBoundObjectsManager arrayMgr;

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
		                      Node::getNamesOfNodes(node.inputs),
		                      Node::getNamesOfNodes(node.outputs),
		                      node.getArgsString());
	}
};
#endif // __CUDACC__
