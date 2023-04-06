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
#include <memory/MemoryKind.hpp>
#include <memory/HostPinnedArray.hpp>
#include <IStreamBound.hpp>
#include <StreamBoundObjectsManager.hpp>

template<typename T>
struct HostPinnedArray;

template <typename T>
struct DeviceAsyncArray : public DeviceArray<MemoryKind::DeviceAsync, T>, public IStreamBound
{
	using Ptr = std::shared_ptr<DeviceAsyncArray<T>>;
	using ConstPtr = std::shared_ptr<const DeviceAsyncArray<T>>;

	CudaStream::Ptr getStream() const override { return stream; }

	void setStream(CudaStream::Ptr newStream) override
	{
		this->memOps = MemoryOperations::get<MemoryKind::DeviceAsync>(newStream);
		this->stream = newStream;
	}

	/**
	 * Safe and synchronous copy implementation.
	 * If needed, an unsafe async version could be implemented;
	 * however, it may be risky since the source memory could be invalidated before the copy is completed.
	 */
	void copyFrom(HostPinnedArray<T>::Ptr src)
	{
		this->resize(src->getCount(), false, false);
		CHECK_CUDA(cudaMemcpyAsync(this->data, src->data, this->getSizeOf() * this->getCount(), cudaMemcpyHostToDevice, this->getStream()->get()));
		CHECK_CUDA(cudaStreamSynchronize(this->getStream()->get()));
	}

	static DeviceAsyncArray<T>::Ptr createWithManager(StreamBoundObjectsManager& manager)
	{
		auto array = createStandalone(manager.getStream());
		manager.registerObject(array);
		return array;
	}

	static DeviceAsyncArray<T>::Ptr createStandalone(CudaStream::Ptr stream)
	{
		return DeviceAsyncArray<T>::Ptr(new DeviceAsyncArray(stream));
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
