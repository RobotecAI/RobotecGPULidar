#include <VArray.hpp>

std::shared_ptr <VArray> VArray::create(rgl_field_t type, std::size_t initialSize)
{
	switch (type) {
		case RGL_FIELD_XYZ_F32: return VArray::create<Vec3f>(initialSize);
		case RGL_FIELD_INTENSITY_F32: return VArray::create<float>(initialSize);
		case RGL_FIELD_RING_ID_U16: return VArray::create<uint16_t>(initialSize);
		case RGL_FIELD_AZIMUTH_F32: return VArray::create<float>(initialSize);
		case RGL_FIELD_DISTANCE_F32: return VArray::create<float>(initialSize);
		case RGL_FIELD_RETURN_TYPE_U8: return VArray::create<uint8_t>(initialSize);
		case RGL_FIELD_TIME_STAMP_F64: return VArray::create<double>(initialSize);
		case RGL_FIELD_PADDING_8: // Intentional fall-through
		case RGL_FIELD_PADDING_16:
		case RGL_FIELD_PADDING_32:
		default: throw std::invalid_argument(fmt::format("VArray does not handle type {}", type));
	}
}

std::shared_ptr<VArray> VArray::clone() const
{ return std::shared_ptr<VArray>(new VArray(*this)); }

void VArray::copyFrom(const void *src, std::size_t bytes)
{
	if ((bytes % sizeOfType) != 0) {
		auto msg = fmt::format("cannot copy {} bytes into an array with element size {}", bytes, sizeOfType);
		throw std::invalid_argument(msg);
	}
	resize(bytes / sizeOfType, false, false);
	CHECK_CUDA(cudaMemcpy(managedData, src, bytes, cudaMemcpyDefault));
}

void VArray::resize(std::size_t newCount, bool zeroInit, bool preserveData)
{
	reserve(newCount, preserveData);
	if (zeroInit) {
		// If data was preserved, zero-init only the new part, otherwise - everything
		char* start = (char*) managedData + sizeOfType * (preserveData ? elemCount : 0);
		std::size_t bytesToClear = sizeOfType * (newCount - (preserveData ? elemCount : 0));
		CHECK_CUDA(cudaMemset(start, 0, bytesToClear));
	}
	elemCount = newCount;
}

void VArray::reserve(std::size_t newCapacity, bool preserveData)
{
	if(elemCapacity >= newCapacity) {
		return;
	}

	void* newMem = nullptr;
	CHECK_CUDA(cudaMallocManaged(&newMem, newCapacity * sizeOfType));

	if (preserveData && managedData != nullptr) {
		CHECK_CUDA(cudaMemcpy(newMem, managedData, sizeOfType * elemCount, cudaMemcpyDefault));
	}

	if (!preserveData) {
		elemCount = 0;
	}

	if (managedData != nullptr) {
		CHECK_CUDA(cudaFree(managedData));
	}

	managedData = newMem;
	elemCapacity = newCapacity;
}

void VArray::hintLocation(int location, cudaStream_t stream)
{
	CHECK_CUDA(cudaMemPrefetchAsync(managedData, elemCapacity * sizeOfType, location, stream));
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
