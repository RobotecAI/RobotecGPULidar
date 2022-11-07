// Copyright 2022 Robotec.AI
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

#include <typeindex>
#include <map>

#include <cuda_runtime.h>
#include <macros/cuda.hpp>

#include <Logger.hpp>
#include <rgl/api/core.h>
#include <math/Vector.hpp>
#include <typingUtils.hpp>

template<typename T>
struct VArrayProxy;

enum struct MemLoc
{
	Host,
	Device
};

/**
 * Dynamically typed, virtual (accessible from GPU & CPU) array.
 */
struct VArray : std::enable_shared_from_this<VArray>
{
	using Ptr = std::shared_ptr<VArray>;
	using ConstPtr = std::shared_ptr<const VArray>;

	// Structure describing allocation instance on a single device (Host / Device)
	struct Instance
	{
		void* data = nullptr;
		int64_t elemCount = 0;
		int64_t elemCapacity = 0;
	};

	// Static construction
	static VArray::Ptr create(rgl_field_t type, std::size_t initialSize=0);

	template<typename T>
	static VArray::Ptr create(std::size_t initialSize=0)
	{ return VArray::Ptr(new VArray(typeid(T), sizeof(T), initialSize)); }


	// Typed proxy construction
	template<typename T>
	typename VArrayProxy<T>::Ptr getTypedProxy()
	{
		if (typeid(T) != typeInfo) {
			auto msg = fmt::format("VArray type mismatch: {} requested as {}", name(typeInfo), name(typeid(T)));
			throw std::invalid_argument(msg);
		}
		return VArrayProxy<T>::create(shared_from_this());
	}

	template<typename T>
	typename VArrayProxy<T>::ConstPtr getTypedProxy() const
	{ return VArrayProxy<T>::create(shared_from_this()); }


	// Methods
	void* getWritePtr(MemLoc location);
	const void* getReadPtr(MemLoc location) const;
	void setData(const void* src, std::size_t elements);
	void resize(std::size_t newCount, bool zeroInit=true, bool preserveData=true);
	void reserve(std::size_t newCapacity, bool preserveData=true);
	std::size_t getElemSize() const { return sizeOfType; }
	std::size_t getElemCount() const { return current().elemCount; }
	std::size_t getElemCapacity() const { return current().elemCapacity; }

	~VArray();

private:
	Instance& current() const { return instance.at(currentLocation); }
	void* memAlloc(std::size_t bytes, std::optional<MemLoc> locationHint= std::nullopt) const;
	void memFree(void*, std::optional<MemLoc> locationHint= std::nullopt) const;
	void migrateToLocation(MemLoc location);

private:
	std::size_t sizeOfType;
	std::reference_wrapper<const std::type_info> typeInfo;

	mutable MemLoc currentLocation;
	mutable std::map<MemLoc, Instance> instance;

	VArray(const std::type_info& type, std::size_t sizeOfType, std::size_t initialSize);

	template<typename T>
	friend struct VArrayTyped;

	template<typename T>
	friend struct VArrayProxy;
};
