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

#include <memory>
#include <cuda.h>

#include <typingUtils.hpp>
#include <CudaStream.hpp>

#include <memory/InvalidArrayCast.hpp>
#include <memory/MemoryOperations.hpp>
#include <memory/IAnyArray.hpp>
#include <memory/MemoryKind.hpp>
#include <memory/IAnyArray.hpp>
#include <memory/MemoryOperations.hpp>

/**
 * Base class implementing resizable Array.
 * Allows for dependency-injection to provide memory operations needed for resizing.
 * @tparam M Allows to statically distinguish Arrays using different memory kinds.
 * @tparam T Type of elements
 */
template<MemoryKind M, typename T>
struct Array : public IAnyArray<M>
{
	static_assert(std::is_trivially_copyable<T>::value);
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
	unsigned long getCount() const override { return count; }
	unsigned long getSizeOf() const override { return sizeof(T); }
	unsigned long getCapacity() const override { return capacity; }
	std::string getTypeName() const override { return name(typeid(T)); }

	void resize(unsigned long newCount, bool zeroInit, bool preserveData) override
	{
		// Ensure capacity
		reserve(newCount, preserveData);

		// Clear expanded part
		if (newCount >= count && zeroInit) {
			memOps.clear(data + count, 0, sizeof(T) * (newCount - count));
		}

		count = newCount;
	}

	void reserve(unsigned long newCapacity, bool preserveData) override
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

	void clear(bool zero) override
	{
		if (data == nullptr) {
			return;
		}

		if (zero) {
			memOps.clear(data, 0, sizeof(T) * count);
		}

		count = 0;
	}

protected:
	unsigned long count = {0 };
	unsigned long capacity = {0 };
	DataType* data = { nullptr };
	MemoryOperations memOps;
};
