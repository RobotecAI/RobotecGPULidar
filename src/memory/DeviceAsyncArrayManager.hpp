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