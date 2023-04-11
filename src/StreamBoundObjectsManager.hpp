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
 */
struct StreamBoundObjectsManager
{
	/**
	 * Registers IStreamBound object.
	 * Registering any object requires prior call to setStream().
	 * The object must be bound with the stream provided to setStream.
	 * @param object Object to register.
	 */
	void registerObject(IStreamBound::Ptr object)
	{
		if (!currentStream.has_value()) {
			auto msg = fmt::format("StreamBoundObjectsManager: attempted to setStream() before call to registerObject()");
			throw std::logic_error(msg);
		}
		if (currentStream.value().expired()) {
			auto msg = fmt::format("StreamBoundObjectsManager: called registerObject(), but the current stream has expired");
			throw std::logic_error(msg);
		}
		if (object->getStream().get() != currentStream.value().lock().get()) {
			auto msg = fmt::format("StreamBoundObjectsManager: attempted to registerObject() using a different stream");
			throw std::logic_error(msg);
		}
		streamBoundObjects.emplace_back(object);
	}

	/**
	 * Ensures that all current and future registered objects will be bound to the given stream.
	 */
	void setStream(CudaStream::Ptr newStream)
	{
		if (!currentStream.has_value()) {
			currentStream = newStream;
			return;
		}
		if (auto lastStreamShPtr = currentStream.value().lock()) {
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
				continue;
			}
			objectSharedPtr->setStream(newStream);
		}
		currentStream = newStream;
	}

	CudaStream::Ptr getStream()
	{
		if (!currentStream.has_value()) {
			auto msg = fmt::format("StreamBoundObjectsManager: getStream() called before setStream()");
			throw std::logic_error(msg);
		}
		if (auto ptr = currentStream.value().lock()) {
			return ptr;
		}
		auto msg = fmt::format("StreamBoundObjectsManager: getStream() called, but the currentStream has expired");
		throw std::logic_error(msg);
	}

private:
	std::optional<std::weak_ptr<CudaStream>> currentStream;
	std::list<std::weak_ptr<IStreamBound>> streamBoundObjects; // List of objects that are bound to currentStream
};
