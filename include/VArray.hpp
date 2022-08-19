#pragma once

#include <typeindex>
#include <cuda_runtime.h>
#include <macros/cuda.hpp>
#include <cuda.h> // TODO Remove

#include <rgl/api/experimental.h>
#include <math/Vector.hpp>
#include <Logger.h>

template<typename T>
struct VArrayProxy;

template<typename T>
struct VArrayTyped;

// VArrayTyped<const> = cannot modify data
// const VArrayTyped<data> = cannot resize, append, reallocate, etc.

struct VArray : std::enable_shared_from_this<VArray>
{
	static constexpr int CPU = -1;
	static constexpr int GPU = 0;

	template<typename T>
	static std::shared_ptr<VArray> create(std::size_t initialSize=0)
	{ return std::shared_ptr<VArray>(new VArray(typeid(T), sizeof(T), initialSize)); }

	static std::shared_ptr<VArray> create(rgl_field_t type, std::size_t initialSize=0)
	{
		switch (type) {
			case RGL_FIELD_XYZ_F32: return VArray::create<Vec3f>(initialSize);
			case RGL_FIELD_XYZP_F32: return VArray::create<Vec4f>(initialSize);
			case RGL_FIELD_INTENSITY_F32: return VArray::create<float>(initialSize);
			case RGL_FIELD_RING_ID_U16: return VArray::create<uint16_t>(initialSize);
			case RGL_FIELD_AZIMUTH_F32: return VArray::create<float>(initialSize);
			case RGL_FIELD_DISTANCE_F32: return VArray::create<float>(initialSize);
			case RGL_FIELD_RETURN_TYPE_U8: return VArray::create<uint8_t>(initialSize);
			case RGL_FIELD_TIME_STAMP_F64: return VArray::create<double>(initialSize);
			case RGL_FIELD_PADDING_8: // Intentional fall-through
			case RGL_FIELD_PADDING_16:
			case RGL_FIELD_PADDING_32:
			default: throw std::invalid_argument(fmt::format("VArray does not handle type {}", type));
		}
	}

	template<typename T>
	std::shared_ptr<VArrayProxy<T>> getTypedProxy()
	{ return VArrayProxy<T>::create(shared_from_this()); }

	template<typename T>
	std::shared_ptr<const VArrayProxy<T>> getTypedProxy() const
	{ return VArrayProxy<T>::create(shared_from_this()); }

	template<typename T>
	std::shared_ptr<VArrayTyped<T>> intoTypedWrapper() &&
	{ return VArrayTyped<T>::create(std::move(*this)); }

	std::shared_ptr<VArray> clone() const
	{ return std::shared_ptr<VArray>(new VArray(*this)); }

	void copyFrom(const void* src, std::size_t bytes)
	{
		if ((bytes % sizeOfType) != 0) {
			auto msg = fmt::format("cannot copy {} bytes into an array with element size {}", bytes, sizeOfType);
			throw std::invalid_argument(msg);
		}
		resize(bytes / sizeOfType, false, false);
		CHECK_CUDA(cudaMemcpy(managedData, src, bytes, cudaMemcpyDefault));
	}

	// TODO zero init
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true)
	{
		reserve(newCount, preserveData);
		if (zeroInit) {
			// If data was preserved, zero-init only the new part, otherwise - everything
			char* start = (char*) managedData + sizeOfType * (preserveData ? elemCount : 0);
			std::size_t bytesToClear = sizeOfType * (newCount - (preserveData ? elemCount : 0));
			CHECK_CUDA(cudaMemset(start, 0, bytesToClear));
		}
		elemCount = newCount;
	}

	void reserve(std::size_t newCapacity, bool preserveData=true)
	{
		if(elemCapacity >= newCapacity) {
			return;
		}

		void* newMem = nullptr;
		CHECK_CUDA(cudaMallocManaged(&newMem, newCapacity * sizeOfType));

		if (preserveData && managedData != nullptr) {
			CHECK_CUDA(cudaMemcpy(newMem, managedData, sizeOfType * elemCount, cudaMemcpyDefault));
		}

		if (!preserveData) {
			elemCount = 0;
		}

		if (managedData != nullptr) {
			CHECK_CUDA(cudaFree(managedData));
		}

		managedData = newMem;
		elemCapacity = newCapacity;
	}

	void hintLocation(int location, cudaStream_t stream=nullptr)
	{
		CHECK_CUDA(cudaMemPrefetchAsync(managedData, elemCapacity * sizeOfType, location, stream));
	}

	// std::size_t getCapacity() const { return elemCapacity; }
	// std::size_t getCount() const { return elemCapacity; } // For now, capacity follows tightly element count.
	// std::size_t getSizeOfType() const { return elemCount * sizeOfType; }
	void* getDevicePtr() const { return managedData; }
	void* getHostPtr(cudaStream_t stream=nullptr, bool prefetch=true) const { return managedData; }


private:
	std::type_index typeIndex;
	std::size_t sizeOfType;
	void *managedData = nullptr;
	std::size_t elemCapacity = 0;
	std::size_t elemCount = 0;

	VArray(const std::type_info& type, std::size_t sizeOfType, std::size_t initialSize)
	: typeIndex(type)
	, sizeOfType(sizeOfType)
	{
		this->resize(initialSize);
	}

	VArray(const VArray& other)
	: typeIndex(other.typeIndex)
	, sizeOfType(other.sizeOfType)
	, elemCapacity(other.elemCapacity)
	, elemCount(other.elemCount)
	{
		CHECK_CUDA(cudaMallocManaged(&managedData, elemCapacity * sizeOfType));
		CHECK_CUDA(cudaMemcpy(managedData, other.managedData, elemCapacity * sizeOfType, cudaMemcpyDefault));
	}

	VArray(VArray&& src)
	: typeIndex(src.typeIndex)
	, sizeOfType(src.sizeOfType)
	, managedData(src.managedData)
	, elemCapacity(src.elemCapacity)
	, elemCount(src.elemCount)
	{
		managedData = nullptr;
		elemCapacity = 0;
		elemCount = 0;
	}

	template<typename T>
	friend struct VArrayTyped;

	template<typename T>
	friend struct VArrayProxy;
};

/*
 * Reference-holding proxy to VArray with a typed interface.
 */
template<typename T>
struct VArrayProxy
{
	using Ptr = std::shared_ptr<VArrayProxy<T>>;
	using ConstPtr = std::shared_ptr<const VArrayProxy<T>>;

	template<typename... Args>
	static std::shared_ptr<VArrayProxy<T>> create(Args... args)
	{ return std::shared_ptr<VArrayProxy<T>>(new VArrayProxy<T>(args...)); }
	static std::shared_ptr<const VArrayProxy<T>> create(std::shared_ptr<const VArray> src)
	{ return std::shared_ptr<const VArrayProxy<T>>(new VArrayProxy<T>(std::const_pointer_cast<VArray>(src))); }

	std::shared_ptr<VArrayProxy<T>> clone() const
	{ return VArrayProxy<T>::create(src->clone()); }

	void copyFrom(const T* srcRaw, std::size_t count) { src->copyFrom(srcRaw, sizeof(T) * count); }

	std::size_t getCount() const { return src->elemCount; }
	std::size_t getCapacity() const { return src->elemCapacity; }
	std::size_t getBytesInUse() const { return src->elemCount * sizeof(T); }
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true) { src->resize(newCount, zeroInit, preserveData); }


	T*          getDevicePtr()         { return reinterpret_cast<T*>(src->getDevicePtr()); }
	T*          getHostPtr()           { return reinterpret_cast<T*>(src->getHostPtr()); }
	CUdeviceptr getCUdeviceptr() const { return reinterpret_cast<CUdeviceptr>(src->getDevicePtr()); }
	const T*    getDevicePtr()   const { return reinterpret_cast<const T*>(src->getDevicePtr()); }
	const T*    getHostPtr()     const { return reinterpret_cast<const T*>(src->getHostPtr()); }
	T&       operator[](int idx)       { return (reinterpret_cast<T*>(src->managedData))[idx]; }
	const T& operator[](int idx) const { return (reinterpret_cast<const T*>(src->managedData))[idx]; }

	void hintLocation(int location, cudaStream_t stream=nullptr) const { src->hintLocation(location, stream); }

private:
	VArrayProxy(std::shared_ptr<VArray> src=nullptr) : src(src == nullptr ? VArray::create<T>() : src) {}
	VArrayProxy(std::size_t initialSize) : src(VArray::create<T>(initialSize)) {}

private:
	std::shared_ptr<VArray> src;
};

/*
 * Owning VArray wrapper with a typed interface.
 */
template<typename T>
struct VArrayTyped
{
	static std::shared_ptr<VArrayTyped<T>> create()	{ return std::shared_ptr<VArrayTyped<T>>(new VArrayTyped<T>());	}
	static std::shared_ptr<VArrayTyped<T>> create(VArray&& src)	{ return std::shared_ptr<VArrayTyped<T>>(new VArrayTyped<T>(src)); }

	// TODO: implement if needed :)

private:
	VArrayTyped(std::size_t initialSize) : src(typeid(T), sizeof(T), initialSize) {}
	explicit VArrayTyped(VArray&& src) : src(std::move(src)) {}

private:
	VArray src;
};
