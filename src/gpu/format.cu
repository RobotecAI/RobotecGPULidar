#include <gpu/trampolines.hpp>

#include <gpu/GPUFieldDesc.hpp>

__global__ void kFormat(size_t fieldCount, GPUFieldDesc* fields, size_t pointCount, size_t pointSize, char* out)
{
	auto tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= pointCount) {
		return;
	}
	// Implement padding
	for (size_t i = 0; i < fieldCount; ++i) {
		memcpy(out + pointSize * tid + fields[i].dstOffset, fields[i].data + fields[i].size * tid, fields[i].size);
	}
}

void gpuFormat(size_t fieldCount, GPUFieldDesc* fields, size_t pointCount, size_t pointSize, char* out)
{
	int threads = 256;
	int blocks = 1 + pointCount / 256;
	kFormat<<<blocks, threads>>>(fieldCount, fields, pointCount, pointSize, out);
}
