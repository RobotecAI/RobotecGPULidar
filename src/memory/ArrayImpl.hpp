#pragma once

#include <memory/Array.hpp>
#include <memory/DeviceAsyncArray.hpp>

template<typename T>
Array<T>::~Array() {
	if (data != nullptr) {
		memOps.deallocate(data);
		data = reinterpret_cast<T*>(0x0000DEAD);
	}
}

template<typename T>
void Array<T>::copyFromExternalRaw(const void *src, size_t srcLength) {
	if (srcLength % getSizeOf() != 0) {
		auto msg = fmt::format("copyFromExternalRaw: buffer size ({}) is not a multiply of element size ({})", srcLength, getSizeOf());
		throw std::runtime_error(msg);
	}
	this->resize(srcLength / getSizeOf(), false, false);

	bool isDstAsync = this->getMemoryKind() == MemoryKind::DeviceAsync;
	CudaStream::Ptr copyStream = isDstAsync
	                             ? this->asSubclass<DeviceAsyncArray>()->getStream()
	                             : CudaStream::getNullStream();
	CHECK_CUDA(cudaMemcpyAsync(data, src, srcLength, cudaMemcpyDefault, copyStream->getHandle()));
	CHECK_CUDA(cudaStreamSynchronize(copyStream->getHandle()));
}

template<typename T>
void Array<T>::copyToExternalRaw(void *dst, size_t dstLength, std::optional<CudaStream::Ptr> alternativeStream) const {
	size_t srcLength = getSizeOf() * getCount();
	if (dstLength < srcLength) {
		auto msg = fmt::format("copyToExternalRaw: dst buffer size ({}) is smaller than src buffer size ({})", dstLength, srcLength);
		throw std::runtime_error(msg);
	}

	if (isHost(this->getMemoryKind())) {
		memcpy(dst, this->data, srcLength);
		return;
	}

	if (!alternativeStream.has_value() && this->getMemoryKind() == MemoryKind::DeviceAsync) {
		auto asyncSrc = this->template asSubclass<DeviceAsyncArray>();
		CHECK_CUDA(cudaStreamSynchronize(asyncSrc->getStream()->getHandle()));
	}

	// For * with alternative stream: alternative stream
	// For DAA without alternative stream: DAA's stream
	// For non-DAA without alternative stream: null stream
	bool isDstAsync = this->getMemoryKind() == MemoryKind::DeviceAsync;
	CudaStream::Ptr copyStream = alternativeStream.has_value()
	                             ? alternativeStream.value()
	                             : (isDstAsync ? this->asSubclass<DeviceAsyncArray>()->getStream() : CudaStream::getNullStream());
	CHECK_CUDA(cudaMemcpyAsync(dst, data, srcLength, cudaMemcpyDefault, copyStream->getHandle()));
	CHECK_CUDA(cudaStreamSynchronize(copyStream->getHandle()));

}

template<typename T>
void Array<T>::copyFrom(Array::ConstPtr src) {
	if (shared_from_this() == src) {
		throw std::runtime_error("attempted to copy Array from itself");
	}
	this->resize(src->getCount(), false, false);
	size_t byteCount = src->getCount() * src->getSizeOf();

	// Both operands are on host - either pageable or pinned.
	// Standard memcpy is faster + avoids the overhead of cudaMemcpy*
	if (isHost(this->getMemoryKind()) && isHost(src->getMemoryKind())) {
		memcpy(this->data, src->data, byteCount);
		return;
	}

	// Ensure src is ready (one day, it can be optimized to some waiting on some cudaEvent, not entire stream)
	bool isSrcAsync = src->getMemoryKind() == MemoryKind::DeviceAsync;
	if (isSrcAsync) {
		auto asyncSrc = src->template asSubclass<DeviceAsyncArray>();
		CHECK_CUDA(cudaStreamSynchronize(asyncSrc->getStream()->getHandle()));
	}

	// Using null stream is hurting performance, but extra safe.
	// TODO: remove null stream usage once DeviceSyncArray is removed
	bool isDstAsync = this->getMemoryKind() == MemoryKind::DeviceAsync;
	CudaStream::Ptr copyStream = isDstAsync
	                           ? this->asSubclass<DeviceAsyncArray>()->getStream()
	                           : isSrcAsync ? src->template asSubclass<DeviceAsyncArray>()->getStream()
	                                        : CudaStream::getNullStream();
	CHECK_CUDA(cudaMemcpyAsync(data, src->data, byteCount, cudaMemcpyDefault, copyStream->getHandle()));
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
	if (newCapacity == 0) {
		throw std::invalid_argument("requested to reserve 0 elements");
	}

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
