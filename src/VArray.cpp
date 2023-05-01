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

#include <VArray.hpp>

#include <RGLFields.hpp>

VArray::VArray(const std::type_info &type, std::size_t sizeOfType, std::size_t initialSize)
: typeInfo(type)
, sizeOfType(sizeOfType)
{
	instance[MemLoc::Host] = {0};
	instance[MemLoc::Device] = {0};
	currentLocation = MemLoc::Device;
	this->resize(initialSize);
}

const void* VArray::getReadPtr(MemLoc location) const
{
	// TODO(prybicki): optimize: do not move if not changed
	if (currentLocation != location) {
		// TODO(prybicki): Refactor it to avoid this hack:
		const_cast<VArray*>(this)->migrateToLocation(location);
	}
	return current().data;
}

void* VArray::getWritePtr(MemLoc location)
{
	return const_cast<void*>(getReadPtr(location));
}



VArray::Ptr VArray::create(rgl_field_t type, std::size_t initialSize)
{
	return createVArray(type, initialSize);
}

void VArray::setData(const void *src, std::size_t elements)
{
	resize(elements, false, false);
	CHECK_CUDA(cudaMemcpy(current().data, src, sizeOfType * elements, cudaMemcpyDefault));
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

void VArray::getData(void* dst, std::size_t elements) const
{
	CHECK_CUDA(cudaMemcpy(dst, current().data, sizeOfType * elements, cudaMemcpyDefault));
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

void VArray::insertData(const void *src, std::size_t elements, std::size_t offset)
{
	reserve(offset + elements, true);
	CHECK_CUDA(cudaMemcpy((void *)((size_t)current().data + sizeOfType * offset), src, sizeOfType * elements, cudaMemcpyDefault));
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
	current().elemCount = std::max(current().elemCount, int64_t(offset + elements));
}

void VArray::resize(std::size_t newCount, bool zeroInit, bool preserveData)
{
	reserve(newCount, preserveData);
	if (zeroInit && current().elemCount < newCount) {
		char* start = (char*) current().data + sizeOfType * current().elemCount;
		std::size_t bytesToClear = sizeOfType * (newCount - current().elemCount);
		CHECK_CUDA(cudaMemset(start, 0, bytesToClear));
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
	}
	current().elemCount = newCount;
}

void VArray::reserve(std::size_t newCapacity, bool preserveData)
{
	if (!preserveData) {
		current().elemCount = 0;
	}

	if(current().elemCapacity >= newCapacity) {
		return;
	}

	void* newMem = memAlloc(newCapacity * sizeOfType);

	if (preserveData && current().data != nullptr) {
		CHECK_CUDA(cudaMemcpy(newMem, current().data, sizeOfType * current().elemCount, cudaMemcpyDefault));
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
	}

	if (current().data != nullptr) {
		memFree(current().data);
	}

	current().data = newMem;
	current().elemCapacity = newCapacity;
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

void VArray::doubleCapacityIfRunningOut(float runningOutThreshold)
{
	if (getElemCapacity() * runningOutThreshold < getElemCount()) {
		reserve(getElemCapacity() * 2, true);
	}
}

VArray::~VArray()
{
	for (auto&& [location, state] : instance) {
		if (state.data != nullptr) {
			memFree(state.data, {location});
			state = {0};
		}
	}
}

void* VArray::memAlloc(std::size_t bytes, std::optional<MemLoc> locationHint) const
{
	MemLoc location = locationHint.has_value() ? locationHint.value() : currentLocation;
	void* ptr = nullptr;
	if (location == MemLoc::Host) {
		CHECK_CUDA(cudaMallocHost(&ptr, bytes));
	}
	if (location == MemLoc::Device) {
		CHECK_CUDA(cudaMalloc(&ptr, bytes));
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
	}
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
	return ptr;
}

void VArray::memFree(void* ptr, std::optional<MemLoc> locationHint) const
{
	MemLoc location = locationHint.has_value() ? locationHint.value() : currentLocation;
	if (location == MemLoc::Host) {
		CHECK_CUDA(cudaFreeHost(ptr));
	}
	if (location == MemLoc::Device) {
		CHECK_CUDA(cudaFree(ptr));
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
	}
	CHECK_CUDA(cudaStreamSynchronize(nullptr));
}

void VArray::migrateToLocation(MemLoc newLoc)
{
	if (currentLocation == newLoc) {
		return;
	}
	void* srcData = current().data;
	size_t elemCount = current().elemCount;
	currentLocation = newLoc;
	setData(srcData, elemCount);
}
