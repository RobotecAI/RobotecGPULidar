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

#include <list>
#include <memory>

#include <IStreamBound.hpp>
#include <memory/ConcreteArrays.hpp>

struct DeviceAsyncArrayManager : public IStreamBound
{
	template<typename T>
	DeviceAsyncArray<T>::Ptr create()
	{
		if (lastStream.expired()) {
			auto msg = fmt::format("DeviceAsyncArrayManager: called create() without prior call to setStream()");
			throw std::logic_error(msg);
		}
		auto ptr = DeviceAsyncArray<T>::create(lastStream.lock());
		streamBoundObjects.emplace_back(ptr);
		return ptr;
	}

	void setStream(CudaStream::Ptr newStream) override
	{
		if (auto lastStreamShPtr = lastStream.lock()) {
			if (lastStreamShPtr.get() == newStream.get()) {
				return;
			}
		}

		// Stream has changed, update bound objects
		auto it = streamBoundObjects.begin();
		for (; it != streamBoundObjects.end(); ++it) {
			auto&& objectWeakPtr = *it;
			if (objectWeakPtr.expired()) {
				it = streamBoundObjects.erase(it);
				continue;
			}
			objectWeakPtr.lock()->setStream(newStream);
		}
		lastStream = newStream;
	}

private:
	std::weak_ptr<CudaStream> lastStream;
	std::list<std::weak_ptr<IStreamBound>> streamBoundObjects; // List of objects that are bound to lastStream
};
