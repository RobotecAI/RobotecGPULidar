#pragma once

#include <typeindex>
#include <cuda_runtime.h>
#include <macros/cuda.hpp>

#include <Logger.h>
#include <rgl/api/experimental.h>
#include <math/Vector.hpp>
#include <typingUtils.hpp>

template<typename T>
struct VArrayProxy;

template<typename T>
struct VArrayTyped;

/**
 * Dynamically typed, virtual (accessible from GPU & CPU) array.
 */
struct VArray : std::enable_shared_from_this<VArray>
{
	static constexpr int CPU = -1;
	static constexpr int GPU = 0;

	template<typename T>
	static std::shared_ptr<VArray> create(std::size_t initialSize=0)
	{ return std::shared_ptr<VArray>(new VArray(typeid(T), sizeof(T), initialSize)); }

	static std::shared_ptr<VArray> create(rgl_field_t type, std::size_t initialSize=0);

	template<typename T>
	std::shared_ptr<VArrayProxy<T>> getTypedProxy()
	{
		if (typeid(T) != typeInfo) {
			auto msg = fmt::format("VArray type mismatch: {} requested as {}", name(typeInfo), name(typeid(T)));
			throw std::invalid_argument(msg);
		}
		return VArrayProxy<T>::create(shared_from_this());
	}

	template<typename T>
	std::shared_ptr<const VArrayProxy<T>> getTypedProxy() const
	{ return VArrayProxy<T>::create(shared_from_this()); }

	template<typename T>
	std::shared_ptr<VArrayTyped<T>> intoTypedWrapper() &&
	{ return VArrayTyped<T>::create(std::move(*this)); }

	std::shared_ptr<VArray> clone() const;
	void copyFrom(const void* src, std::size_t bytes);
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true);
	void reserve(std::size_t newCapacity, bool preserveData=true);
	void hintLocation(int location, cudaStream_t stream) const;
	inline void* getDevicePtr() const { return managedData; }
	inline void* getHostPtr() const { return managedData; }
	inline std::size_t getElemSize() const { return sizeOfType; }

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
