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

#include <memory/DeviceArray.hpp>
#include <memory/HostPinnedArray.hpp>
#include <memory/MemoryKind.hpp>

template <typename T>
struct DeviceSyncArray : public DeviceArray<MemoryKind::DeviceSync, T>
{
	using Ptr = std::shared_ptr<DeviceSyncArray<T>>;
	using ConstPtr = std::shared_ptr<const DeviceSyncArray<T>>;
	using DeviceArray<MemoryKind::DeviceSync, T>::copyFrom;

	static DeviceSyncArray<T>::Ptr create()
	{
		return DeviceSyncArray<T>::Ptr(new DeviceSyncArray(MemoryOperations::get<MemoryKind::DeviceSync>()));
	}

	void copyFrom(HostPinnedArray<T>::ConstPtr src)
	{
		this->resize(src->getCount(), false, false);
		CHECK_CUDA(cudaMemcpy(this->data, src->data, sizeof(T) * src->getCount(), cudaMemcpyHostToDevice));
	}

	void copyFromHost(const T* hostSrc, std::size_t count)
	{
		this->resize(count, false, false);
		CHECK_CUDA(cudaMemcpy(this->data, hostSrc, sizeof(T) * count, cudaMemcpyHostToDevice));
	}

protected:
	using DeviceArray<MemoryKind::DeviceSync, T>::DeviceArray;
};

