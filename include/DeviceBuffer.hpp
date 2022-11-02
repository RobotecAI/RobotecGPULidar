#pragma once

#pragma once

#include <cuda.h>
#include <vector>
#include <type_traits>
#include <optional>

#include <Logger.hpp>
#include <HostPinnedBuffer.hpp>

#include <macros/cuda.hpp>

// TODO: elemCount and elemCapacity are duplictes..
template<typename T>
struct HostPinnedBuffer;

template <typename T>
struct DeviceBuffer
{
	typedef T ValueType;
private:
	static_assert(std::is_trivially_copyable<T>::value, "DeviceBuffer is instantiable only for types that can be copied between CPU and GPU");

	T* data {nullptr};
	std::size_t elemCount {0};
	std::size_t elemCapacity {0};

public:
	DeviceBuffer(DeviceBuffer&) = delete;
	DeviceBuffer(DeviceBuffer&&) = delete;
	DeviceBuffer& operator=(DeviceBuffer&) = delete;
	DeviceBuffer& operator=(DeviceBuffer&&) = delete;

	DeviceBuffer() { }

	~DeviceBuffer()
	{
		if (data != nullptr) {
			cudaFree(data);
			data = nullptr;
		}
	}

	void copyFromHost(const T* src, std::size_t srcElemCount)
	{
		ensureDeviceCanFit(srcElemCount);
		CHECK_CUDA(cudaMemcpy(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice));
		elemCount = srcElemCount;
	}

	void copyFromHostAsync(const T* src, std::size_t srcElemCount, cudaStream_t stream)
	{
		ensureDeviceCanFit(srcElemCount);
		CHECK_CUDA(cudaMemcpyAsync(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice, stream));
		elemCount = srcElemCount;
	}

	void copyFromHost(const HostPinnedBuffer<int>& src)
	{
		copyFromHost(src.readHost(), src.getElemCount());
	}

	void copyFromHost(const std::vector<T>& src)
	{
		copyFromHost(src.data(), src.size());
	}

	const T* readDevice() const
	{
		return data;
	}

	T* writeDevice()
	{
		return data;
	}

	CUdeviceptr readDeviceRaw() const
	{
		return reinterpret_cast<CUdeviceptr>(
				reinterpret_cast<void*>(
						const_cast<T*>(readDevice())
				)
		);
	}

	std::size_t getElemCount() const { return elemCount; }
	std::size_t getSizeOfElem() const { return sizeof(T); }
	std::size_t getByteSize() const { return getElemCount() * getSizeOfElem(); }

	bool resizeToFit(std::size_t newElemCount, bool clear=false)
	{
		bool resized = ensureDeviceCanFit(newElemCount);
		elemCount = newElemCount;
		if (clear) {
			CHECK_CUDA(cudaMemset(data, 0, elemCount * sizeof(T)));
		}
		return resized;
	}

private:
	bool ensureDeviceCanFit(std::size_t newElemCount) {
		if (newElemCount == 0) {
			auto msg = fmt::format("Attempted to allocate {} bytes of memory", newElemCount);
			throw std::logic_error(msg);
		}
		if (elemCapacity >= newElemCount) {
			return false;
		}
		if (data != nullptr) {
			CHECK_CUDA(cudaFree(data));
		}
		CHECK_CUDA(cudaMalloc(reinterpret_cast<void**>(&data), newElemCount * sizeof(T)));
		elemCapacity = newElemCount;
		return true;
	}

	friend struct fmt::formatter<DeviceBuffer<T>>;
};
