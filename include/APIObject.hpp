#pragma once

#include <set>
#include <memory>

/**
 * Objects shared through C-API should inherit from APIObject<T>, which:
 * - Tracks instances, which may be helpful to e.g. debug leaks on the client side.
 * - Disallows to make stack instantiations, which cannot be reliably returned through C-API
 * - Disables automatic copying and moving
 */
template<typename T>
struct APIObject
{
	static std::set<std::shared_ptr<APIObject<T>>> instances;

	template<typename... Args>
	static std::shared_ptr<T> create(Args&&... args)
	{
		auto ptr = std::shared_ptr<T>(new T(std::forward<Args>(args)...));
		instances.insert(ptr);
		return ptr;
	}

	static void release(std::shared_ptr<APIObject<T>> toDestroy)
	{
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
std::set<std::shared_ptr<APIObject<T>>> APIObject<T>::instances; \
template struct APIObject<Type>
