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
#include <stdexcept>

#include "IStreamBound.hpp"

/**
 * Convenience class that allows changing bound stream of registered objects.
 * By default, StreamBoundObjectsManager bounds registered objects to the NULL stream.
 */
struct StreamBoundObjectsManager
{
	/**
	 * Registers IStreamBound object.
	 * @param object Object to register.
	 */
	void registerObject(IStreamBound::Ptr object)
	{
		if (currentStream.expired()) {
			auto msg = fmt::format("StreamBoundObjectsManager: called registerObject(), but the current stream has expired");
			throw std::logic_error(msg);
		}
		if (auto currentStreamLocked = currentStream.lock(); object->getStream().get() != currentStreamLocked.get()) {
			object->setStream(currentStreamLocked);
		}
		streamBoundObjects.emplace_back(object);
	}

	/**
	 * Ensures that all current and future registered objects will be bound to the given stream.
	 */
	void setStream(CudaStream::Ptr newStream)
	{
		// Fast path, stream did not change.
		if (auto lastStreamShPtr = currentStream.lock()) {
			if (lastStreamShPtr.get() == newStream.get()) {
				return;
			}
		}

		// Stream has changed, update bound objects
		auto it = streamBoundObjects.begin();
		for (; it != streamBoundObjects.end(); ++it) {
			auto&& objectWeakPtr = *it;
			auto objectSharedPtr = objectWeakPtr.lock();
			if (objectSharedPtr == nullptr) {
				it = streamBoundObjects.erase(it);
				if (it == streamBoundObjects.end()) {
					// Avoid incrementing end iterator, breaks on Windows
					break;
				}
				continue;
			}
			objectSharedPtr->setStream(newStream);
		}
		currentStream = newStream;
	}

	CudaStream::Ptr getStream()
	{
		if (auto ptr = currentStream.lock()) {
			return ptr;
		}
		auto msg = fmt::format("StreamBoundObjectsManager: getStream() called, but the currentStream has expired");
		throw std::logic_error(msg);
	}

private:
	std::weak_ptr<CudaStream> currentStream = CudaStream::getNullStream();
	std::list<std::weak_ptr<IStreamBound>> streamBoundObjects; // List of objects that are bound to currentStream
};
