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
#include <memory/MemoryKind.hpp>
#include <memory/InvalidArrayCast.hpp>
#include <memory/HostPageableArray.hpp>

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

	// Explanation why: https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
	// Note: do not repeat this for methods, since it inhibits virtual dispatch mechanism
	using Array<M, T>::data;
	using Array<M, T>::count;
	using Array<M, T>::capacity;

	CUdeviceptr getDeviceReadPtr() const { return reinterpret_cast<CUdeviceptr>(this->getReadPtr()); }
	CUdeviceptr getDeviceWritePtr() { return getDeviceReadPtr(); }
protected:
	using Array<M, T>::Array;
};
