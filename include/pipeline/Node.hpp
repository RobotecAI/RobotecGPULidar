#pragma once

#include <memory>
#include <vector>

#include <VArray.hpp>
#include <rgl/api/experimental.h>
#include <APIObject.hpp>

struct Node : APIObject<Node>, std::enable_shared_from_this<Node>
{
	Node() = default;

	void setParent(std::shared_ptr<Node> parent)
	{
		this->parent = parent;
		if (parent != nullptr) {
			parent->children.push_back(shared_from_this());
		}
	}

	virtual void validate() = 0;
	virtual void execute() = 0;
	virtual ~Node() = default;

	std::shared_ptr<Node> parent {};
	std::vector<std::shared_ptr<Node>> children {};
protected:
	template<typename T>
	std::shared_ptr<T> validateParent()
	{
		if (parent == nullptr) {
			auto msg = fmt::format("{} requires parent, but has none", name(typeid(*this)));
			throw InvalidPipeline(msg);
		}
		std::shared_ptr<T> parentTyped = std::dynamic_pointer_cast<T>(parent);
		if (parentTyped == nullptr) {
			auto msg = fmt::format("{} requires parent of type {}, but {} was found",
			                       name(typeid(*this)), name(typeid(T)), name(typeid(*parent)));
			throw InvalidPipeline(msg);
		}
		return parentTyped;
	}
};
