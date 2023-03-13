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

#include <memory/IAnyArray.hpp>
#include <memory/InvalidArrayCast.hpp>
#include <memory/MemoryOperations.hpp>

/**
 * Base class implementing resizable Array.
 * Allows for dependency-injection to provide memory operations needed for resizing.
 * @tparam T Type of elements
 */
template<typename T>
struct Array : public IAnyArray
{
	static_assert(std::is_trivially_copyable<T>::value);
	using Ptr = std::shared_ptr<Array<T>>;
	using ConstPtr = std::shared_ptr<const Array<T>>;
	using DataType = T;

	virtual ~Array();
	Array(const Array<T>&) = delete;
	Array(Array<T>&&) = delete;
	Array<T>& operator=(const Array<T>&) = delete;
	Array<T>& operator=(Array<T>&&) = delete;

	/**
	 * Attempts casting into a subclass.
	 */
	template<template <typename> typename Subclass>
	Subclass<T>::Ptr asSubclass();

	/** Const overload **/
	template<template <typename> typename Subclass>
	Subclass<T>::ConstPtr asSubclass() const;

	const void* getRawReadPtr() const override { return data; }
	void* getRawWritePtr() override { return data; }
	std::size_t getCount() const override { return count; }
	std::size_t getSizeOf() const override { return sizeof(T); }
	std::size_t getCapacity() const override { return capacity; }

	void resize(std::size_t newCount, bool zeroInit, bool preserveData) override;
	void reserve(std::size_t newCapacity, bool preserveData) override;
	void clear(bool zero) override;

	void copyFromExternal(const T *src, size_t srcCount);

protected:
	std::size_t count = {0 };
	std::size_t capacity = {0 };
	DataType* data = { nullptr };
	MemoryOperations memOps;

protected:
	// Array should not be instanced directly. Use concrete subclasses.
	Array(MemoryOperations memOps) : IAnyArray(typeid(T)), memOps(std::move(memOps)) {}
};

// Order matters
#include <memory/HostArray.inl>
#include <memory/HostPinnedArray.inl>
#include <memory/HostPageableArray.inl>
#include <memory/DeviceArray.inl>
#include <memory/DeviceSyncArray.inl>
#include <memory/DeviceAsyncArray.inl>
#include <memory/ArrayImpl.inl>
