#pragma once

#include <typeindex>
#include <cuda_runtime.h>
#include <macros/cuda.hpp>

template<typename T>
struct VArrayTyped;

struct VArray : std::enable_shared_from_this<VArray>
{
	template<typename T>
	static std::shared_ptr<VArray> create()
	{
		return std::shared_ptr<VArray>(new VArray(typeid(T), sizeof(T)));
	}

	template<typename T>
	std::shared_ptr<VArrayTyped<T>> getTypedInterface()
	{
		return VArrayTyped<T>::create(shared_from_this());
	}

	void copyFrom(const void* src, std::size_t bytes)
	{
		ensureCapacity(bytes);
		CHECK_CUDA(cudaMemcpy(managedData, src, bytes, cudaMemcpyDefault));
	}

private:
	std::type_index typeIndex;
	std::size_t sizeOfType;
	void *managedData = nullptr;
	std::size_t capacity = 0;

	void ensureCapacity(std::size_t bytesNeeded)
	{
		if(capacity >= bytesNeeded) {
			return;
		}

		if (managedData != nullptr) {
			CHECK_CUDA(cudaFree(managedData));
			managedData = nullptr;
			capacity = 0;
		}
		CHECK_CUDA(cudaMallocManaged(&managedData, bytesNeeded));
		capacity = bytesNeeded;
	}

	VArray(const std::type_info& type, std::size_t sizeOfType) : typeIndex(type), sizeOfType(sizeOfType) {}
};

template<typename T>
struct VArrayTyped
{
	static std::shared_ptr<VArrayTyped<T>> create(std::shared_ptr<VArray> src=nullptr)
	{
		if (src == nullptr) {
			src = VArray::create<T>();
		}
		return std::shared_ptr<VArrayTyped<T>>(new VArrayTyped<T>(src));
	}

	void copyFrom(const T* srcRaw, std::size_t count)
	{
		src->copyFrom(srcRaw, sizeof(T) * count);
	}

private:
	VArrayTyped(std::shared_ptr<VArray> src) : src(src) {}

private:
	std::shared_ptr<VArray> src;
};
