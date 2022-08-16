#pragma once

#include <memory>
#include <vector>

#include <VArray.hpp>
#include <rgl/api/experimental.h>
#include <APIObject.hpp>

struct Node : APIObject<Node>, std::enable_shared_from_this<Node>
{
	using Ptr = std::shared_ptr<Node>;

	Node() = default;

	void setParent(Node::Ptr parent)
	{
		if (parent == nullptr) {
			auto msg = fmt::format("attempted to set an empty parent for {}", getNodeTypeName());
			throw InvalidPipeline(msg);
		}
		this->inputs.push_back(parent);
		if (parent != nullptr) {
			parent->outputs.push_back(shared_from_this());
		}
	}

	virtual void validate() = 0;
	virtual void schedule(cudaStream_t stream) = 0;
	virtual ~Node() = default;

	std::vector<Node::Ptr> inputs {};
	std::vector<Node::Ptr> outputs {};

	std::string getNodeTypeName() const { return name(typeid(*this)); }

	template <template <typename _> typename Container>
	static std::string getNodeTypeNames(const Container<Node::Ptr>& nodes, std::string_view separator=", ")
	{
		std::string output{};
		for (auto&& node: nodes) {
			output += node->getNodeTypeName();
			output += separator;
		}
		if (!output.empty()) {
			std::string_view view {output};
			view.remove_suffix(separator.size()); // Remove trailing separator
			output = std::string(view);
		}
		return output;
	}

	template<typename T, template<typename _> typename Container>
	static std::vector<std::shared_ptr<T>> filter(const Container<Node::Ptr>& nodes)
	{
		std::vector<std::shared_ptr<T>> typedNodes {};
		for (auto&& node : nodes) {
			auto typedNode = std::dynamic_pointer_cast<T>(node);
			if (typedNode != nullptr) {
				typedNodes.push_back(typedNode);
			}
		}
		return typedNodes;
	}

	template<typename T, template<typename _> typename Container>
	static std::shared_ptr<T> getExactlyOne(const Container<Node::Ptr>& nodes)
	{
		std::vector<std::shared_ptr<T>> typedNodes = Node::filter<T>(nodes);
		if (typedNodes.size() != 1) {
			auto msg = fmt::format("looked for {}, but found [{}]", name(typeid(T)), getNodeTypeNames(nodes));
			throw InvalidPipeline(msg);
		}
		return typedNodes[0];
	}

protected:
	template<typename T>
	std::shared_ptr<T> getValidInput()
	{
		return getExactlyOne<T>(inputs);
	}
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
		return fmt::format_to(ctx.out(), "{}(in=[{}], out=[{}])",
		                      node.getNodeTypeName(),
		                      Node::getNodeTypeNames(node.inputs),
		                      Node::getNodeTypeNames(node.outputs));
	}
};
#endif // __CUDACC__