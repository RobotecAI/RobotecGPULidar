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

#pragma once

#include <type_traits>
#include <optional>
#include "Logger.hpp"
#include "DeviceBuffer.hpp"
#include <macros/cuda.hpp>

template<typename T>
struct DeviceBuffer;

template<typename T>
struct HostPinnedBuffer
{
private:
	static_assert(std::is_trivially_copyable<T>::value,
	              "HostPinnedBuffer is instantiable only for types that can be copied between CPU and GPU");

	T* data{ nullptr };
	std::size_t elemCount{ 0 };
	std::size_t elemCapacity{ 0 };

public:
	HostPinnedBuffer(HostPinnedBuffer&) = delete;
	HostPinnedBuffer(HostPinnedBuffer&&) = delete;
	HostPinnedBuffer& operator=(HostPinnedBuffer&) = delete;
	HostPinnedBuffer& operator=(HostPinnedBuffer&&) = delete;

	HostPinnedBuffer() {}

	~HostPinnedBuffer()
	{
		if (data != nullptr) {
			cudaFreeHost(data);
		}
	}

	void copyFromDeviceAsync(const DeviceBuffer<T>& src, cudaStream_t stream)
	{
		ensureHostCanFit(src.getElemCount());
		CHECK_CUDA(cudaMemcpyAsync(data, src.readDevice(), src.getElemCount() * sizeof(T), cudaMemcpyDeviceToHost, stream));
		elemCount = src.getElemCount();
	}

	const T* readHost() const { return data; }

	std::size_t getElemCount() const { return elemCount; }
	std::size_t getSizeOfElem() const { return sizeof(T); }
	std::size_t getByteSize() const { return getElemCount() * getSizeOfElem(); }

private:
	void ensureHostCanFit(std::size_t newElemCount)
	{
		if (newElemCount == 0) {
			auto msg = fmt::format("Attempted to allocate {} bytes of memory", newElemCount);
			throw std::logic_error(msg);
		}
		if (elemCapacity >= newElemCount) {
			return;
		}
		if (data != nullptr) {
			CHECK_CUDA(cudaFreeHost(data));
		}
		CHECK_CUDA(cudaMallocHost(reinterpret_cast<void**>(&data), newElemCount * sizeof(T)));
		elemCapacity = newElemCount;
	}
};
