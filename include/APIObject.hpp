#pragma once

#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <spdlog/fmt/fmt.h>

#include <typingUtils.hpp>
#include <RGLExceptions.hpp>

/**
 * Objects shared through C-API should inherit from APIObject<T>, which:
 * - Tracks instances, which may be helpful to e.g. debug leaks on the client side.
 * - Disallows to make stack instantiations, which cannot be reliably returned through C-API
 * - Disables automatic copying and moving
 */
template<typename T>
struct APIObject
{
	static std::unordered_map<APIObject<T>*, std::shared_ptr<T>> instances;

	// Constructs T instance + 2 shared_ptrs:
	// - One is stored in instances to account for non-C++ usage
	//   - This must be manually deleted via release(T*)
	// - One is returned and can be:
	//   - .get()-ed to return to non-C++ code
	//   - passed within C++ code (as a convenience for e.g. testing)
	template<typename... Args>
	static std::shared_ptr<T> create(Args&&... args)
	{
		return create<T>(args...);
	}

	template<typename SubClass, typename... Args>
	static std::shared_ptr<SubClass> create(Args&&... args)
	{
		// Cannot use std::make_shared due to private constructor
		auto ptr = std::shared_ptr<SubClass>(new SubClass(std::forward<Args>(args)...));
		instances.insert({ptr.get(), ptr});
		return ptr;
	}

	// Translates raw pointers coming from C-api into shared object
	// This allows to detect / prevent user from:
	// - Use-after-free
	// - Passing invalid handle type (e.g. mesh instead of entity)
	// In both cases 'this' will be not found in 'instances'.
	static std::shared_ptr<T> validatePtr(T* rawPtr)
	{
		auto it = instances.find(rawPtr);
		if (it == instances.end()) {
			auto msg = fmt::format("RGL API Error: Object does not exist: {} {}", name(typeid(T)), reinterpret_cast<void*>(rawPtr));
			throw InvalidAPIObject(msg);
		}
		return it->second;
	}

	template<typename SubClass>
	static std::shared_ptr<SubClass> validatePtr(T* rawPtr)
	{
		auto node = validatePtr(rawPtr);
		auto subclass = std::dynamic_pointer_cast<SubClass>(node);
		if (subclass != nullptr) {
			return subclass;
		}
		auto msg = fmt::format("RGL API Error: Node type mismatch: expected {}, got {}", name(typeid(SubClass)), name(typeid(*node)));
		throw InvalidAPIObject(msg);

	}

	static void release(T* toDestroy)
	{
		validatePtr(toDestroy);
		instances.erase(toDestroy);
	}

	APIObject(APIObject<T>&) = delete;
	APIObject(APIObject<T>&&) = delete;
	APIObject<T>& operator=(APIObject<T>&) = delete;
	APIObject<T>& operator=(APIObject<T>&&) = delete;
protected:
	APIObject() = default;
};

// This should be used in .cpp file to make an instance of static variable(s) of APIObject<Type>
#define API_OBJECT_INSTANCE(Type)                \
template<typename T>                             \
std::unordered_map<APIObject<T>*, std::shared_ptr<T>> APIObject<T>::instances; \
template struct APIObject<Type>
