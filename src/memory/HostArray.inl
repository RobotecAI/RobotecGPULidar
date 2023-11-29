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

/**
 * HostArray extends Array with some convenience methods that are only possible for host memory.
 * @tparam T See base class
 */
template<typename T>
struct HostArray : public Array<T>
{
	using Ptr = std::shared_ptr<HostArray<T>>;
	using ConstPtr = std::shared_ptr<const HostArray<T>>;

	T* getWritePtr() { return data; }
	const T* getReadPtr() const { return data; }

	/** Appends given value at the end of Array, similar to std::vector<T>::push_back */
	void append(T value);

	/**
	 * Accesses data with checking index bounds.
	 * Throws std::out_of_range if the index is invalid.
	 */
	T& at(size_t idx);

	/** Const overload */
	const T& at(size_t idx) const { return const_cast<HostArray<T>*>(this)->at(idx); }

	/** Accesses data without checking index bounds */
	T& operator[](size_t idx) { return data[idx]; }

	/** Const overload */
	const T& operator[](size_t idx) const { return data[idx]; }

protected:
	// Explanation why: https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
	// Note: do not repeat this for methods, since it inhibits virtual dispatch mechanism
	using Array<T>::Array;
	using Array<T>::data;
	using Array<T>::count;
	using Array<T>::capacity;
};
