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

#include <cuda.h>

#include <typingUtils.hpp>
#include <CudaStream.hpp>

#include <memory/Array.hpp>
#include <memory/IAnyArray.hpp>
#include <memory/InvalidArrayCast.hpp>
#include <memory/MemoryKind.hpp>

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
