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
#include <IStreamBound.hpp>
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

	/**
	 * Replaces current data with data from src.
	 */
	void copyFrom(IAnyArray::ConstPtr src)
	{
		if (shared_from_this() == src) {
			throw std::runtime_error("attempted to copy Array from itself");
		}
		if (this->typeIndex != src->typeIndex) {
			throw std::runtime_error("attempted to copy from Array with different data type");
		}
		this->resize(src->getCount(), false, false);
		this->insertAt(src, 0);
	}
	/**
	 * Appends data from src to current data.
	 */
	void appendFrom(IAnyArray::ConstPtr src)
	{
		if (shared_from_this() == src) {
			throw std::runtime_error("attempted to append Array from itself");
		}
		if (this->typeIndex != src->typeIndex) {
			throw std::runtime_error("attempted to append from Arrays with different data type");
		}
		std::size_t n = this->getCount() + src->getCount();
		// Find next power of 2
		n -= 1;
		n |= n >> 1;
		n |= n >> 2;
		n |= n >> 4;
		n |= n >> 8;
		n |= n >> 16;
		n |= n >> 32;
		n += 1;
		this->reserve(n, true);
		this->resize(this->getCount() + src->getCount(), false, true);
		this->insertAt(src, this->getCount());
	}

	virtual ~IAnyArray() = default;

protected:
	IAnyArray(std::type_index typeIndex) : typeIndex(typeIndex) {}

private:
	void insertAt(IAnyArray::ConstPtr src, std::size_t skipCount)
	{
		size_t offset = skipCount * src->getSizeOf();
		size_t byteCount = src->getCount() * src->getSizeOf();
		void* writePtr = reinterpret_cast<char*>(this->getRawWritePtr()) + offset;

		// Both operands are on host - either pageable or pinned.
		// Standard memcpy is faster + avoids the overhead of cudaMemcpy*
		if (isHost(this->getMemoryKind()) && isHost(src->getMemoryKind())) {
			memcpy(writePtr, src->getRawReadPtr(), byteCount);
			return;
		}

		// Ensure src is ready (one day, it can be optimized to some waiting on some cudaEvent, not entire stream)
		auto srcStreamBound = std::dynamic_pointer_cast<const IStreamBound>(src);
		if (srcStreamBound != nullptr) {
			CHECK_CUDA(cudaStreamSynchronize(srcStreamBound->getStream()->getHandle()));
		}

		// Using null stream is hurting performance, but extra safe.
		// TODO: remove null stream usage once DeviceSyncArray is removed
		auto dstStreamBound = std::dynamic_pointer_cast<IStreamBound>(shared_from_this());
		CudaStream::Ptr copyStream = dstStreamBound != nullptr
		                             ? dstStreamBound->getStream()
		                             : srcStreamBound != nullptr ? srcStreamBound->getStream()
		                                                         : CudaStream::getNullStream();
		CHECK_CUDA(cudaMemcpyAsync(writePtr, src->getRawReadPtr(), byteCount, cudaMemcpyDefault, copyStream->getHandle()));
		CHECK_CUDA(cudaStreamSynchronize(copyStream->getHandle()));
	}

	std::type_index typeIndex;
};
