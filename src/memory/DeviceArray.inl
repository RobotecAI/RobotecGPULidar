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

/**
 * DeviceArray extends Array with some convenience methods useful for dealing with device memory.
 * @tparam T See base class
 */
template <typename T>
struct DeviceArray : public Array<T>
{
	using Ptr = std::shared_ptr<DeviceArray<T>>;
	using ConstPtr = std::shared_ptr<const DeviceArray<T>>;

	T* getWritePtr() { return data; }
	const T* getReadPtr() const { return data; }
	CUdeviceptr getDeviceReadPtr() const { return reinterpret_cast<CUdeviceptr>(this->getReadPtr()); }
	CUdeviceptr getDeviceWritePtr() { return getDeviceReadPtr(); }

protected:
	// Explanation why: https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
	// Note: do not repeat this for methods, since it inhibits virtual dispatch mechanism
	using Array<T>::Array;
	using Array<T>::data;
	using Array<T>::count;
	using Array<T>::capacity;
};
