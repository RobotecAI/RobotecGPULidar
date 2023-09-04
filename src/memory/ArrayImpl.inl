#pragma once

#include <memory/Array.hpp>
#include <macros/handleDestructorException.hpp>

template<typename T>
Array<T>::~Array() try {
	if (data != nullptr) {
		memOps.deallocate(data);
		data = reinterpret_cast<T*>(0x0000DEAD);
	}
}
HANDLE_DESTRUCTOR_EXCEPTION

template<typename T>
void Array<T>::copyFromExternal(const T *src, size_t srcCount) {
	this->resize(srcCount, false, false);

	auto dstStreamBound = std::dynamic_pointer_cast<IStreamBound>(shared_from_this());
	CudaStream::Ptr copyStream = dstStreamBound != nullptr
	                             ? dstStreamBound->getStream()
	                             : CudaStream::getNullStream();
	CHECK_CUDA(cudaMemcpyAsync(this->getRawWritePtr(), src, srcCount * sizeof(T), cudaMemcpyDefault, copyStream->getHandle()));
	CHECK_CUDA(cudaStreamSynchronize(copyStream->getHandle()));
}

template<typename T>
void Array<T>::clear(bool zero) {
	if (data == nullptr) {
		return;
	}

	if (zero) {
		memOps.clear(data, 0, sizeof(T) * count);
	}

	count = 0;
}

template<typename T>
void Array<T>::reserve(unsigned long newCapacity, bool preserveData) {
	if (!preserveData) {
		count = 0;
	}

	if(capacity >= newCapacity) {
		return;
	}

	T* newMem = reinterpret_cast<T*>(memOps.allocate(sizeof(T) * newCapacity));

	if (preserveData && data != nullptr) {
		memOps.copy(newMem, data, sizeof(T) * count);
	}

	if (data != nullptr) {
		memOps.deallocate(data);
	}

	data = newMem;
	capacity = newCapacity;
}

template<typename T>
void Array<T>::resize(unsigned long newCount, bool zeroInit, bool preserveData) {
	// Ensure capacity
	reserve(newCount, preserveData);

	// Clear expanded part
	if (newCount >= count && zeroInit) {
		memOps.clear(data + count, 0, sizeof(T) * (newCount - count));
	}

	count = newCount;
}

template<typename T>
template<template<typename> typename Subclass>
Subclass<T>::ConstPtr Array<T>::asSubclass() const {
	if (auto ptr = std::dynamic_pointer_cast<const Subclass<T>>(this->shared_from_this())) {
		return ptr;
	}
	THROW_INVALID_ARRAY_CAST(const Subclass<T>);
}

template<typename T>
template<template<typename> typename Subclass>
Subclass<T>::Ptr Array<T>::asSubclass() {
	if (auto ptr = std::dynamic_pointer_cast<Subclass<T>>(this->shared_from_this())) {
		return ptr;
	}
	THROW_INVALID_ARRAY_CAST(Subclass<T>);
}

/** HOST ARRAY **/

template<typename T>
T &HostArray<T>::at(size_t idx) {
	if (idx >= count) {
		auto msg = fmt::format("index out of range: {}/{}", idx, count);
		throw std::out_of_range(msg);
	}
	return data[idx];
}

template<typename T>
void HostArray<T>::append(T value) {
	if (count + 1 > capacity) {
		auto newCapacity = (capacity > 0)
		                   ? (capacity * 2)
		                   : 1;
		this->reserve(newCapacity, true);
	}
	data[count] = std::move(value);
	count += 1;
}
