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
#include <typeindex>

#include <memory/MemoryKind.hpp>
#include <memory/InvalidArrayCast.hpp>
#include <typingUtils.hpp>

template<typename T>
struct Array;

/**
 * Dynamically typed handle for Array<M, T> and its subclasses.
 */
struct IAnyArray : public std::enable_shared_from_this<IAnyArray>
{
	using Ptr = std::shared_ptr<IAnyArray>;
	using ConstPtr = std::shared_ptr<const IAnyArray>;

	/**
	 * Method to convert from statically typed array to dynamically typed array.
	 */
	IAnyArray::Ptr asAny() { return IAnyArray::shared_from_this(); }

	/** Const overload */
	IAnyArray::ConstPtr asAny() const { return IAnyArray::shared_from_this(); }

	/**
	 * Method to convert from dynamically typed array to statically typed array.
	 * https://en.cppreference.com/w/cpp/language/template_parameters
	 */
	template<typename T>
	Array<T>::Ptr asTyped()
	{
		if (auto ptr = std::dynamic_pointer_cast<Array<T>>(this->shared_from_this())) {
			return ptr;
		}
		THROW_INVALID_ARRAY_CAST(Array<T>);
	}

	/** Const overload */
	template<typename T>
	Array<T>::ConstPtr asTyped() const
	{
		if (auto ptr = std::dynamic_pointer_cast<const Array<T>>(this->shared_from_this())) {
			return ptr;
		}
		THROW_INVALID_ARRAY_CAST(Array<T>);
	}

	/**
	 * @return Pointer to raw readable data. Use as a last resort and provide reasoning.
	 */
	virtual const void* getRawReadPtr() const = 0;

	/**
	 * @return Pointer to raw writable data. Use as a last resort and provide reasoning.
	 */
	virtual void* getRawWritePtr() = 0;

	/**
	 * Copies all data to the given external pageable memory.
	 * If alternativeStream has value, then copy will be scheduled in that stream.
	 * In such case, if Array<T> is DeviceAsyncArray<T>, its bound stream will be not synchronized.
	 * This kind of usage is reserved for situations when callee ensures proper synchronization.
	 * Callee is responsible for ensuring that the bound stream has no pending operations that involves this DAA.
	 * (E.g. if there's a pending resize, pointer values already changed, but memory is not yet allocated, so the copy will fail)
	 */
	virtual void copyToExternalRaw(void* dst, size_t length, std::optional<CudaStream::Ptr> alternativeStream) const = 0;

	/**
	 * Copies all elements from external (pageable) memory.
	 * After the function returns, data has been fully copied (no asynchronicity).
	 */
	virtual void copyFromExternalRaw(const void* src, size_t length) = 0;

	/**
	 * @return MemoryKind determining actual subclass.
	 */
	virtual MemoryKind getMemoryKind() const = 0;

	/**
	 * @return Number of elements already in the array.
	 */
	virtual std::size_t getCount() const = 0;

	/**
	 * @return Number of elements that array can fit before reallocation.
	 */
	virtual std::size_t getCapacity() const = 0;

	/**
	 * @return Size in bytes of each array element.
	 */
	virtual std::size_t getSizeOf() const = 0;

	/**
	 * Expands or shrinks the array, possibly erasing existing elements. It does not reduce capacity.
	 * @param newCount Desired element count after operation.
	 * @param zeroInit If true, elements will be zero-initialized. Otherwise, elements may have garbage values.
	 * @param preserveData If false, resize will skip copying old data as an optimization.
	 * Useful when the data would be overwritten anyway.
	 */
	virtual void resize(std::size_t newCount, bool zeroInit, bool preserveData) = 0;

	/**
	 * Ensures that the array can fit given number of elements, including existing ones.
	 * @param newCapacity Minimal number of elements that array will fit after the operation.
	 * @param preserveData If false, reserve will skip copying old data as an optimization. Useful when the data would be overwritten anyway.
	 */
	virtual void reserve(std::size_t newCapacity, bool preserveData) = 0;

	/**
	 * Removes all elements, but keeps existing allocation.
	 * @param zero If true, existing elements will be set to zero.
	 */
	virtual void clear(bool zero) = 0;

	virtual ~IAnyArray() = default;
};
