#pragma once

#include <typeindex>

#include <cuda_runtime.h>
#include <macros/cuda.hpp>

#include <Logger.hpp>
#include <rgl/api/core.h>
#include <math/Vector.hpp>
#include <typingUtils.hpp>

template<typename T>
struct VArrayProxy;

template<typename T>
struct VArrayTyped;

enum struct MemLoc
{
	Host,
	Device
};

/**
 * Dynamically typed, virtual (accessible from GPU & CPU) array.
 */
struct VArray : std::enable_shared_from_this<VArray>
{
	using Ptr = std::shared_ptr<VArray>;
	using ConstPtr = std::shared_ptr<const VArray>;

	// struct State
	// {
	// 	void* data = nullptr;
	// 	int64_t version = 0;
	// 	int64_t elemCount = 0;
	// 	int64_t elemCapacity = 0;
	// };

	// Static construction
	static VArray::Ptr create(rgl_field_t type, std::size_t initialSize=0);

	template<typename T>
	static VArray::Ptr create(std::size_t initialSize=0)
	{ return VArray::Ptr(new VArray(typeid(T), sizeof(T), initialSize)); }


	// Typed proxy construction
	template<typename T>
	typename VArrayProxy<T>::Ptr getTypedProxy()
	{
		if (typeid(T) != typeInfo) {
			auto msg = fmt::format("VArray type mismatch: {} requested as {}", name(typeInfo), name(typeid(T)));
			throw std::invalid_argument(msg);
		}
		return VArrayProxy<T>::create(shared_from_this());
	}

	template<typename T>
	typename VArrayProxy<T>::ConstPtr getTypedProxy() const
	{ return VArrayProxy<T>::create(shared_from_this()); }


	// Methods
	VArray::Ptr clone() const;
	void* getWritePtr(MemLoc location);
	const void* getReadPtr(MemLoc location) const;
	void copyFrom(const void* src, std::size_t elements);
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true);
	void reserve(std::size_t newCapacity, bool preserveData=true);
	inline std::size_t getElemSize() const { return sizeOfType; }
	inline std::size_t getCount() const { return elemCount; }
	~VArray() { cudaFree(managedData); managedData = nullptr; }

private:
	std::reference_wrapper<const std::type_info> typeInfo;
	std::size_t sizeOfType;
	void *managedData = nullptr;
	std::size_t elemCapacity = 0;
	std::size_t elemCount = 0;

	VArray(const std::type_info& type, std::size_t sizeOfType, std::size_t initialSize);
	VArray(const VArray& other);
	VArray(VArray&& src);

	template<typename T>
	friend struct VArrayTyped;

	template<typename T>
	friend struct VArrayProxy;
};
