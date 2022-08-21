#include <gpu/nodeKernels.hpp>
#include <gpu/GPUFieldDesc.hpp>
#include <macros/cuda.hpp>
#include <vector>

template<typename Kernel, typename... KernelArgs>
void run(Kernel&& kernel, size_t threads, KernelArgs... kernelArgs)
{
	int blockDim = 256;
	int blockCount = 1 + threads / 256;
	void* args[] = {&threads, &kernelArgs...};
	CHECK_CUDA(cudaLaunchKernel(reinterpret_cast<void*>(kernel), blockCount, blockDim, args, 0, 0));
}

__global__ void kFormat(size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc* fields, char* out)
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

__global__ void kTransformRays(size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{
	auto tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= rayCount) {
		return;
	}
	outRays[tid] = inRays[tid] * transform;
}

void gpuFormat(size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc *fields, char *out)
{ run(kFormat, pointCount, pointSize, fieldCount, fields, out); }

void gpuTransformRays(size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{ run(kTransformRays, rayCount, inRays, outRays, transform); };

