#include <VArray.hpp>

#include <RGLFields.hpp>

VArray::Ptr VArray::create(rgl_field_t type, std::size_t initialSize)
{
	return createVArray(type, initialSize);
}

VArray::Ptr VArray::clone() const
{ return VArray::Ptr(new VArray(*this)); }

void VArray::copyFrom(const void *src, std::size_t elements)
{
	resize(elements, false, false);
	CHECK_CUDA(cudaMemcpy(managedData, src, sizeOfType * elements, cudaMemcpyDefault));
}

void VArray::resize(std::size_t newCount, bool zeroInit, bool preserveData)
{
	// If !preserveData, reserve(...) zeroes elemCount
	reserve(newCount, preserveData);
	if (zeroInit && elemCount < newCount) {
		char* start = (char*) managedData + sizeOfType * elemCount;
		std::size_t bytesToClear = sizeOfType * (newCount - elemCount);
		CHECK_CUDA(cudaMemset(start, 0, bytesToClear));
	}
	elemCount = newCount;
}

void VArray::reserve(std::size_t newCapacity, bool preserveData)
{
	if (!preserveData) {
		elemCount = 0;
	}

	if(elemCapacity >= newCapacity) {
		return;
	}

	void* newMem = nullptr;
	CHECK_CUDA(cudaMallocManaged(&newMem, newCapacity * sizeOfType));

	if (preserveData && managedData != nullptr) {
		CHECK_CUDA(cudaMemcpy(newMem, managedData, sizeOfType * elemCount, cudaMemcpyDefault));
	}

	if (managedData != nullptr) {
		CHECK_CUDA(cudaFree(managedData));
	}

	managedData = newMem;
	elemCapacity = newCapacity;
}

VArray::VArray(const std::type_info &type, std::size_t sizeOfType, std::size_t initialSize)
: typeInfo(type)
, sizeOfType(sizeOfType)
{
	this->resize(initialSize);
}

VArray::VArray(const VArray &other)
: typeInfo(other.typeInfo)
, sizeOfType(other.sizeOfType)
, elemCapacity(other.elemCapacity)
, elemCount(other.elemCount)
{
	CHECK_CUDA(cudaMallocManaged(&managedData, elemCapacity * sizeOfType));
	CHECK_CUDA(cudaMemcpy(managedData, other.managedData, elemCapacity * sizeOfType, cudaMemcpyDefault));
}

VArray::VArray(VArray &&src)
: typeInfo(src.typeInfo)
, sizeOfType(src.sizeOfType)
, managedData(src.managedData)
, elemCapacity(src.elemCapacity)
, elemCount(src.elemCount)
{
	managedData = nullptr;
	elemCapacity = 0;
	elemCount = 0;
}
