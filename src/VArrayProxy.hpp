// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

	VArray::Ptr untyped() { return src; };
	VArray::ConstPtr untyped() const { return src; };

	void setData(const T* srcRaw, std::size_t count) { src->setData(srcRaw, count); }
	void getData(T* dstRaw, std::size_t count) const { src->getData(dstRaw, count); }

	std::size_t getCount() const { return src->getElemCount(); }
	std::size_t getCapacity() const { return src->getElemCapacity(); }
	std::size_t getBytesInUse() const { return src->getElemCount() * sizeof(T); }
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true) { src->resize(newCount, zeroInit, preserveData); }

	T* getWritePtr(MemLoc location) { return reinterpret_cast<T*>(src->getWritePtr(location)); }
	const T* getReadPtr(MemLoc location) const { return reinterpret_cast<const T*>(src->getReadPtr(location)); }
	CUdeviceptr getCUdeviceptr() { return reinterpret_cast<CUdeviceptr>(src->getWritePtr(MemLoc::Device)); }

	// TODO(prybicki): remove these in favor of ...(location)
	T*          getHostPtr()           { return reinterpret_cast<T*>(src->getWritePtr(MemLoc::Host)); }
	const T*    getHostPtr()     const { return reinterpret_cast<const T*>(src->getReadPtr(MemLoc::Device)); }
	T*          getDevicePtr()         { return reinterpret_cast<T*>(src->getWritePtr(MemLoc::Device)); }
	const T*    getDevicePtr()   const { return reinterpret_cast<const T*>(src->getReadPtr(MemLoc::Device)); }

	T&       operator[](int idx)       { return (reinterpret_cast<T*>(src->getWritePtr(MemLoc::Host)))[idx]; }
	const T& operator[](int idx) const { return (reinterpret_cast<const T*>(src->getReadPtr(MemLoc::Host)))[idx]; }

private:
	VArrayProxy(VArray::Ptr src=nullptr) : src(src == nullptr ? VArray::create<T>() : src) {}
	VArrayProxy(std::size_t initialSize) : src(VArray::create<T>(initialSize)) {}

private:
	VArray::Ptr src;
};
