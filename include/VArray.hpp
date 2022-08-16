#pragma once

#include <typeindex>
#include <cuda_runtime.h>
#include <macros/cuda.hpp>

#include <rgl/api/experimental.h>
#include <math/Vector.hpp>

template<typename T>
struct VArrayTyped;

struct VArray : std::enable_shared_from_this<VArray>
{
	template<typename T>
	static std::shared_ptr<VArray> create()
	{
		return std::shared_ptr<VArray>(new VArray(typeid(T), sizeof(T)));
	}

	static std::shared_ptr<VArray> create(rgl_field_t type)
	{
		switch (type) {
			case RGL_FIELD_XYZ_F32: return VArray::create<Vec3f>();
			case RGL_FIELD_INTENSITY_F32: return VArray::create<float>();
			case RGL_FIELD_RING_ID_U16: return VArray::create<uint16_t>();
			case RGL_FIELD_AZIMUTH_F32: return VArray::create<float>();
			case RGL_FIELD_DISTANCE_F32: return VArray::create<float>();
			case RGL_FIELD_RETURN_TYPE_U8: return VArray::create<uint8_t>();
			case RGL_FIELD_TIME_STAMP_F64: return VArray::create<double>();
			case RGL_FIELD_PADDING_8: // Intentional fall-through
			case RGL_FIELD_PADDING_16:
			case RGL_FIELD_PADDING_32:
			default: throw std::invalid_argument(fmt::format("VArray does not handle type {}", type));
		}
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

	void ensureCapacity(std::size_t elementsNeeded)
	{
		std::size_t bytesNeeded = elementsNeeded * sizeOfType;
		if(elemCapacity >= bytesNeeded) {
			return;
		}

		if (managedData != nullptr) {
			CHECK_CUDA(cudaFree(managedData));
			managedData = nullptr;
			elemCapacity = 0;
		}
		CHECK_CUDA(cudaMallocManaged(&managedData, bytesNeeded));
		elemCapacity = elementsNeeded;
	}

	std::size_t getCapacity() const { return elemCapacity; }

private:
	std::type_index typeIndex;
	std::size_t sizeOfType;
	void *managedData = nullptr;
	std::size_t elemCapacity = 0;


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

	std::size_t size() const { return src->getCapacity(); }

private:
	VArrayTyped(std::shared_ptr<VArray> src) : src(src) {}

private:
	std::shared_ptr<VArray> src;
};
