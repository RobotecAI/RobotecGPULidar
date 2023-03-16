// Copyright 2023 Robotec.AI
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

#include <CudaStream.hpp>
#include <memory/MemoryKind.hpp>
#include <memory/IAnyArray.hpp>
#include <memory/MemoryOperations.hpp>
#include <memory/InvalidArrayCast.hpp>
#include <typingUtils.hpp>
#include <cuda.h>

/**
 * Base class implementing resizable Array.
 * Allows for dependency-injection to provide memory operations needed for resizing.
 * @tparam M Allows to statically distinguish Arrays using different memory kinds.
 * @tparam T Type of elements
 */
template<MemoryKind M, typename T>
struct Array : public IAnyArray<M>
{
	static_assert(std::is_trivial<T>::value);
	using Ptr = std::shared_ptr<Array<M, T>>;
	using ConstPtr = std::shared_ptr<const Array<M, T>>;
	using DataType = T;

protected:
	// Array should not be instanced directly. Use concrete subclasses.
	Array(MemoryOperations memOps) : memOps(std::move(memOps)) {}

public:
	virtual ~Array()
	{
		if (data != nullptr) {
			memOps.deallocate(data);
			data = reinterpret_cast<T*>(0x0000DEAD);
		}
	}

	Array(const Array<M, T>&) = delete;
	Array(Array<M, T>&&) = delete;
	Array<M,T>& operator=(const Array<M, T>&) = delete;
	Array<M,T>& operator=(Array<M, T>&&) = delete;

public:
	T* getWritePtr() { return data; }
	const T* getReadPtr() const { return data; }
	size_t getCount() const override { return count; }
	size_t getSizeOf() const override { return sizeof(T); }
	size_t getCapacity() const override { return capacity; }
	std::string getTypeName() const override { return name(typeid(T)); }

	void resize(std::size_t newCount, bool zeroInit, bool preserveData) override
	{
		// Ensure capacity
		reserve(newCount, preserveData);

		// Clear expanded part
		if (newCount >= count && zeroInit) {
			memOps.clear(data + count, 0, sizeof(T) * (newCount - count));
		}

		count = newCount;
	}

	void reserve(std::size_t newCapacity, bool preserveData) override
	{
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

protected:
	size_t count = { 0 };
	size_t capacity = { 0 };
	DataType* data = { nullptr };
	MemoryOperations memOps;
};

/**
 * DeviceArray extends Array with some convenience methods useful for dealing with device memory.
 * @tparam M See base class
 * @tparam T See base class
 */
template <MemoryKind M, typename T>
struct DeviceArray : public Array<M, T>
{
	static_assert(M == MemoryKind::DeviceSync || M == MemoryKind::DeviceAsync);
	using Ptr = std::shared_ptr<DeviceArray<M, T>>;
	using ConstPtr = std::shared_ptr<const DeviceArray<M, T>>;

	void copyFrom(Array<MemoryKind::HostPageable, T>::ConstPtr src)
	{
		this->resize(src->getCount(), false, false);
		CHECK_CUDA(cudaMemcpy(this->data, src->getReadPtr(), sizeof(T) * this->getCount(), cudaMemcpyHostToDevice));
	}

	CUdeviceptr getDeviceReadPtr() const { return reinterpret_cast<CUdeviceptr>(this->getReadPtr()); }
	CUdeviceptr getDeviceWritePtr() { return getDeviceReadPtr(); }
protected:
	using Array<M, T>::Array;
};

/**
 * HostArray extends Array with some convenience methods that are only possible for host memory.
 * @tparam M See base class
 * @tparam T See base class
 */
template<MemoryKind M, typename T>
struct HostArray : public Array<M, T>
{
	static_assert(M == MemoryKind::HostPageable || M == MemoryKind::HostPinned);
	using Ptr = std::shared_ptr<HostArray<M, T>>;
	using ConstPtr = std::shared_ptr<const HostArray<M, T>>;

	// Explanation why: https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
	// Note: do not repeat this for methods, since it inhibits virtual dispatch mechanism
	using Array<M, T>::data;
	using Array<M, T>::count;
	using Array<M, T>::capacity;

	/** Appends given value at the end of Array, similar to std::vector<T>::push_back */
	void append(T value)
	{
		if (count + 1 > capacity) {
			auto newCapacity = (capacity > 0)
			                   ? (capacity * 2)
			                   : 1;
			this->reserve(newCapacity, true);
		}
		data[count] = std::move(value);
		count += 1;
	}

	/**
	 * Accesses data with checking index bounds.
	 * Throws std::out_of_range if the index is invalid.
	 */
	T& at(size_t idx)
	{
		if (idx >= count) {
			auto msg = fmt::format("index out of range: {}/{}", idx, count);
			throw std::out_of_range(msg);
		}
		return data[idx];
	}

	/** Const overload */
	virtual const T& at(size_t idx) const { return const_cast<HostArray<M, T>*>(this)->at(idx); }

	/** Accesses data without checking index bounds */
	T& operator[](size_t idx) { return data[idx]; }

	/** Const overload */
	const T& operator[](size_t idx) const {return data[idx]; }
protected:
	using Array<M, T>::Array;
};
