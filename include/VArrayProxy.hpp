#pragma once

#include <VArray.hpp>
#include <cuda.h>

/*
 * Reference-holding proxy to VArray with a typed interface.
 */
template<typename T>
struct VArrayProxy
{
	using Ptr = std::shared_ptr<VArrayProxy<T>>;
	using ConstPtr = std::shared_ptr<const VArrayProxy<T>>;

	template<typename... Args>
	static VArrayProxy<T>::Ptr create(Args... args)
	{ return VArrayProxy<T>::Ptr(new VArrayProxy<T>(args...)); }
	static VArrayProxy<T>::ConstPtr create(std::shared_ptr<const VArray> src)
	{ return VArrayProxy<T>::ConstPtr(new VArrayProxy<T>(std::const_pointer_cast<VArray>(src))); }

	VArrayProxy<T>::Ptr clone() const
	{ return VArrayProxy<T>::create(src->clone()); }

	VArray::Ptr untyped() { return src; };
	VArray::ConstPtr untyped() const { return src; };

	void copyFrom(const T* srcRaw, std::size_t count) { src->copyFrom(srcRaw, count); }

	std::size_t getCount() const { return src->elemCount; }
	std::size_t getCapacity() const { return src->elemCapacity; }
	std::size_t getBytesInUse() const { return src->elemCount * sizeof(T); }
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true) { src->resize(newCount, zeroInit, preserveData); }

	T*          getHostPtr()           { return reinterpret_cast<T*>(src->getWritePtr(MemLoc::host())); }
	const T*    getHostPtr()     const { return reinterpret_cast<const T*>(src->getReadPtr(MemLoc::device())); }
	T*          getDevicePtr()         { return reinterpret_cast<T*>(src->getWritePtr(MemLoc::device())); }
	const T*    getDevicePtr()   const { return reinterpret_cast<const T*>(src->getReadPtr(MemLoc::device())); }
	CUdeviceptr getCUdeviceptr() const { return reinterpret_cast<CUdeviceptr>(src->getWritePtr(MemLoc::device())); }

	T&       operator[](int idx)       { return (reinterpret_cast<T*>(src->managedData))[idx]; }
	const T& operator[](int idx) const { return (reinterpret_cast<const T*>(src->managedData))[idx]; }

private:
	VArrayProxy(VArray::Ptr src=nullptr) : src(src == nullptr ? VArray::create<T>() : src) {}
	VArrayProxy(std::size_t initialSize) : src(VArray::create<T>(initialSize)) {}

private:
	VArray::Ptr src;
};
