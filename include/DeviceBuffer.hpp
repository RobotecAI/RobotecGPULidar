#pragma once

#pragma once

#include <cuda.h>
#include <vector>
#include <type_traits>
#include <optional>

#include <Logger.h>
#include <HostPinnedBuffer.hpp>
#include <data_types/PointTypes.h>
#include <utils/optix_macros.h>

#include <macros/cuda.hpp>

// TODO: elemCount and elemCapacity are duplictes..
template<typename T>
struct HostPinnedBuffer;

template <typename T>
struct DeviceBuffer
{
	typedef T ValueType;
private:
	static_assert(std::is_trivially_copyable<T>::value, "DeviceBuffer is instantiable only for types that can be copied between Host and GPU");

	T* data {nullptr};
	std::size_t elemCount {0};
	std::size_t elemCapacity {0};
	std::string name;

public:
	DeviceBuffer(DeviceBuffer&) = delete;
	DeviceBuffer(DeviceBuffer&&) = delete;
	DeviceBuffer& operator=(DeviceBuffer&) = delete;
	DeviceBuffer& operator=(DeviceBuffer&&) = delete;

	DeviceBuffer(const std::string& name = "<unnamed>") : name(name)
	{
		TRACE("{}::DeviceBuffer()", name);
	}

	~DeviceBuffer()
	{
		TRACE("{}::~DeviceBuffer()", name);
		if (data != nullptr) {
			cudaFree(data);
			data = nullptr;
		}
	}

	void copyFromHost(const T* src, std::size_t srcElemCount)
	{
		TRACE("{}::copyFromHost(src={}, srcElemCount={})", name, (void*) src, srcElemCount);
		ensureDeviceCanFit(srcElemCount);
		CHECK_CUDA(cudaMemcpy(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice));
		elemCount = srcElemCount;
	}

	void copyFromHostAsync(const T* src, std::size_t srcElemCount, cudaStream_t stream)
	{
		TRACE("{}::copyFromHostAsync(src={}, srcElemCount={}, stream={})", name, (void*) src, srcElemCount, (void*) stream);
		ensureDeviceCanFit(srcElemCount);
		CHECK_CUDA(cudaMemcpyAsync(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice, stream));
		elemCount = srcElemCount;
	}

	void copyFromHost(const HostPinnedBuffer<int>& src)
	{
		TRACE("{}::copyFromHost<HPB>(hpbName={}, hpbSize)", name, src.getName(), src.getElemCount());
		copyFromHost(src.readHost(), src.getElemCount());
	}

	void copyFromHost(const std::vector<T>& src)
	{
		TRACE("{}::copyFromHost<vector>(srcData={}, srcSize={})", src.size(), (void*) src.data(), src.size());
		copyFromHost(src.data(), src.size());
	}

	const T* readDevice() const
	{
		TRACE("{}::readDevice()");
		return data;
	}

	T* writeDevice()
	{
		TRACE("{}::writeDevice()", name);
		return data;
	}

	CUdeviceptr readDeviceRaw() const
	{
		TRACE("{}::readDeviceRaw()");
		return reinterpret_cast<CUdeviceptr>(
				reinterpret_cast<void*>(
						const_cast<T*>(readDevice())
				)
		);
	}

	std::size_t getElemCount() const { return elemCount; }

	std::size_t getByteSize() const { return getElemCount() * sizeof(T); }

	void resizeToFit(std::size_t newElemCount, bool clear=false)
	{
		TRACE("{}::resizeToFit(newElemCount={}, clear={})", name, newElemCount, clear);
		ensureDeviceCanFit(newElemCount);
		elemCount = newElemCount;
		if (clear) {
			CHECK_CUDA(cudaMemset(data, 0, elemCount * sizeof(T)));
		}
	}

	std::string getName() const { return name; }

private:
	void ensureDeviceCanFit(std::size_t newElemCount) {
		if (newElemCount == 0) {
			auto msg = fmt::format("Attempted to allocate {} bytes of memory", newElemCount);
			throw std::logic_error(msg);
		}
		if (elemCapacity >= newElemCount) {
			return;
		}
		if (data != nullptr) {
			CHECK_CUDA(cudaFree(data));
		}
		CHECK_CUDA(cudaMalloc(reinterpret_cast<void**>(&data), newElemCount * sizeof(T)));
		elemCapacity = newElemCount;
	}

	friend struct fmt::formatter<DeviceBuffer<T>>;
};
