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

#include <memory/HostArray.hpp>
#include <memory/MemoryKind.hpp>
#include <memory/DeviceAsyncArray.hpp>

template <typename T>
struct HostPinnedArray : public HostArray<MemoryKind::HostPinned, T>
{
	using Ptr = std::shared_ptr<HostPinnedArray<T>>;
	using ConstPtr = std::shared_ptr<const HostPinnedArray<T>>;

	static HostPinnedArray<T>::Ptr create()
	{
		return HostPinnedArray<T>::Ptr {
			new HostPinnedArray(MemoryOperations::get<MemoryKind::HostPinned>())
		};
	}

	void copyFrom(DeviceAsyncArray<T>::Ptr source)
	{
		this->resize(source->getCount(), false, false);
		CHECK_CUDA(cudaMemcpyAsync(this->data, source->data, this->getSizeOf() * this->getCount(), cudaMemcpyDeviceToHost,
		                           source->getStream()->getHandle()));
		CHECK_CUDA(cudaStreamSynchronize(source->getStream()->getHandle()));
	}

protected:
	using HostArray<MemoryKind::HostPinned, T>::HostArray;
};
