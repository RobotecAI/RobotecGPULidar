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

#include <memory/IStreamBound.hpp>
#include <memory/DeviceArray.hpp>
#include <memory/MemoryKind.hpp>

template <typename T>
struct DeviceAsyncArray : public DeviceArray<MemoryKind::DeviceAsync, T>, public IStreamBound
{
	using Ptr = std::shared_ptr<DeviceAsyncArray<T>>;
	using ConstPtr = std::shared_ptr<const DeviceAsyncArray<T>>;

	void setStream(CudaStream::Ptr newStream) override
	{
		this->memOps = MemoryOperations::get<MemoryKind::DeviceAsync>(newStream);
	}

	static DeviceAsyncArray<T>::Ptr create(CudaStream::Ptr streamArg)
	{
		return DeviceAsyncArray<T>::Ptr(new DeviceAsyncArray(MemoryOperations::get<MemoryKind::DeviceAsync>(streamArg)));
	}

protected:
	DeviceAsyncArray(CudaStream::Ptr streamArg)
	  : DeviceArray<MemoryKind::DeviceAsync, T>(MemoryOperations::get<MemoryKind::DeviceAsync>(streamArg))
	  , stream(streamArg) {}

protected:
	CudaStream::Ptr stream; // Needed to implement IStreamBound
	using DeviceArray<MemoryKind::DeviceAsync, T>::DeviceArray;

	friend struct DeviceAsyncArrayManager;
};
