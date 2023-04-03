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


/**
 * Dynamically typed handle for Array<M, T> and its subclasses.
 * @tparam M Kind of memory. @see MemoryKind
 */
template<MemoryKind M>
struct IAnyArray : public std::enable_shared_from_this<IAnyArray<M>>
{
	using Ptr = std::shared_ptr<IAnyArray<M>>;
	using ConstPtr = std::shared_ptr<const IAnyArray<M>>;

	/**
	 * Method to convert from statically typed array to dynamically typed array.
	 */
	IAnyArray<M>::Ptr asAnyArray() { return IAnyArray<M>::shared_from_this(); }

	/** Const overload */
	IAnyArray<M>::ConstPtr asAnyArray() const { return IAnyArray<M>::shared_from_this(); }

	/**
	 * Method to convert from dynamically typed array to statically typed array.
	 * https://en.cppreference.com/w/cpp/language/template_parameters
	 */
	template<typename T, template<typename> typename Subclass>
	Subclass<T>::Ptr asTypedArray()
	{
		static_assert(std::is_base_of<IAnyArray<M>, Subclass<T>>::value);
		if (auto ptr = std::dynamic_pointer_cast<Subclass<T>>(this->shared_from_this())) {
			return ptr;
		}
		auto msg = fmt::format("InvalidArrayCast: {} -> {}", this->getTypeName(), name(typeid(T)));
		throw InvalidArrayCast(msg);
	}

	// Const overload
	template<typename T, template<typename> typename Subclass>
	Subclass<T>::ConstPtr asTypedArray() const
	{
		static_assert(std::is_base_of<IAnyArray<M>, Subclass<T>>::value);
		if (auto ptr = std::dynamic_pointer_cast<const Subclass<T>>(this->shared_from_this())) {
			return ptr;
		}
		auto msg = fmt::format("InvalidArrayCast: {} -> {}", this->getTypeName(), name(typeid(T)));
		throw InvalidArrayCast(msg);
	}

	/**
	 * @return Number of elements already in the array.
	 */
	virtual std::size_t getCount() const = 0;

	/**
	 * @return Number of elements that array can fit before reallocation.
	 */
	virtual std::size_t getCapacity() const = 0;

	/**
	 * @return Type name of the contained element.
	 */
	virtual std::string getTypeName() const = 0;

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
