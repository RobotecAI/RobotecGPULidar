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
	virtual ~Node() = default;
	void addParent(Node::Ptr parent);

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

	inline std::string getName() const { return name(typeid(*this)); }

	const std::vector<Node::Ptr>& getInputs() const { return inputs; }
	const std::vector<Node::Ptr>& getOutputs() const { return outputs; }

	const bool isActive() const { return is_active; }
	void setActive(bool active) { is_active = active; }

protected:
	template <template <typename _> typename Container>
	static std::string getNodeTypeNames(const Container<Node::Ptr>& nodes, std::string_view separator=", ")
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

	template<typename T, template<typename _> typename Container>
	static std::vector<typename T::Ptr> filter(const Container<Node::Ptr>& nodes)
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

	template<typename T, template<typename _> typename Container>
	static typename T::Ptr getExactlyOne(const Container<Node::Ptr>& nodes)
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

	void prependNode(Node::Ptr node);

protected:
	bool is_active {true};
	std::vector<Node::Ptr> inputs {};
	std::vector<Node::Ptr> outputs {};

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
		return fmt::format_to(ctx.out(), "{}(in=[{}], out=[{}])",
		                      node.getName(),
		                      Node::getNodeTypeNames(node.inputs),
		                      Node::getNodeTypeNames(node.outputs));
	}
};
#endif // __CUDACC__