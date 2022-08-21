#pragma once

#include <VArray.hpp>

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

	void hintLocation(int location, cudaStream_t stream) const { src->hintLocation(location, stream); }

private:
	VArrayProxy(std::shared_ptr<VArray> src=nullptr) : src(src == nullptr ? VArray::create<T>() : src) {}
	VArrayProxy(std::size_t initialSize) : src(VArray::create<T>(initialSize)) {}

private:
	std::shared_ptr<VArray> src;
};
