#include <gpu/nodeKernels.hpp>
#include <gpu/GPUFieldDesc.hpp>
#include <macros/cuda.hpp>
#include <vector>

#include <thrust/device_ptr.h>
#include <thrust/scan.h>

template<typename Kernel, typename... KernelArgs>
void run(Kernel&& kernel, cudaStream_t stream, size_t threads, KernelArgs... kernelArgs)
{
	int blockDim = 256;
	int blockCount = 1 + threads / 256;
	void* args[] = {&threads, &kernelArgs...};
	CHECK_CUDA(cudaLaunchKernel(reinterpret_cast<void*>(kernel), blockCount, blockDim, args, 0, stream));
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

__global__ void kApplyCompaction(size_t pointCount, size_t fieldSize, const RGLField<RGL_FIELD_IS_HIT_I32>::Type* shouldWrite, const CompactionIndexType*writeIndex, char *dst, const char *src)
{
	int32_t rIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (rIdx >= pointCount) {
		return;
	}
	if (!shouldWrite[rIdx]) {
		return;
	}
	printf("Writing %d\n", rIdx);
	int wIdx = writeIndex[rIdx] - 1;
	memcpy(dst + fieldSize * wIdx, src + fieldSize * rIdx, fieldSize);
}

void gpuFormat(cudaStream_t stream, size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc *fields, char *out)
{ run(kFormat, stream, pointCount, pointSize, fieldCount, fields, out); }

void gpuTransformRays(cudaStream_t stream, size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{ run(kTransformRays, stream, rayCount, inRays, outRays, transform); };

void gpuFindCompaction(cudaStream_t stream, size_t pointCount, const RGLField<RGL_FIELD_IS_HIT_I32>::Type* isHit, CompactionIndexType* hitCountInclusive, size_t* outHitCount)
{
	// beg and end could be used as const pointers, however thrust does not support it
	auto beg = thrust::device_ptr<const int32_t>(isHit);
	auto end = thrust::device_ptr<const int32_t>(isHit + pointCount);
	auto dst = thrust::device_ptr<int32_t>(hitCountInclusive);

	// Note: this will compile only in a .cu file
	thrust::inclusive_scan(thrust::cuda::par.on(stream), beg, end, dst);
	CHECK_CUDA(cudaMemcpyAsync(outHitCount, hitCountInclusive + pointCount - 1, sizeof(*hitCountInclusive), cudaMemcpyDefault, stream));
}

void gpuApplyCompaction(cudaStream_t stream, size_t pointCount, size_t fieldSize, const RGLField<RGL_FIELD_IS_HIT_I32>::Type* shouldWrite, const CompactionIndexType *writeIndex, char *dst, const char *src)
{ run(kApplyCompaction, stream, pointCount, fieldSize, shouldWrite, writeIndex, dst, src); }

