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

#include <gpu/kernelUtils.hpp>
#include <gpu/nodeKernels.hpp>
#include <gpu/GPUFieldDesc.hpp>
#include <macros/cuda.hpp>
#include <vector>

#include <thrust/device_ptr.h>
#include <thrust/scan.h>

__global__ void kFormatSoaToAos(size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc *soaInData, char *aosOutData)
{
	LIMIT(pointCount);
	for (size_t i = 0; i < fieldCount; ++i) {
		memcpy(aosOutData + pointSize * tid + soaInData[i].dstOffset, soaInData[i].readDataPtr + soaInData[i].size * tid, soaInData[i].size);
	}
}

__global__ void kFormatAosToSoa(size_t pointCount, size_t pointSize, size_t fieldCount, const char* aosInData, GPUFieldDesc* soaOutData)
{
	LIMIT(pointCount);
	for (size_t i = 0; i < fieldCount; ++i) {
		memcpy(soaOutData[i].writeDataPtr + soaOutData[i].size * tid, aosInData + pointSize * tid + soaOutData[i].dstOffset, soaOutData[i].size);
	}
}

__global__ void kTransformRays(size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{
	LIMIT(rayCount);
	outRays[tid] = transform * inRays[tid];
}

__global__ void kVelocityDistortRays(size_t rayCount, const Mat3x4f* inRays, const float* offsets, size_t offsetsCount, const Vec3f sensorLinearVelocity, const Vec3f sensorAngularVelocity, Mat3x4f* outRays)
{
	LIMIT(rayCount);
	outRays[tid] = Mat3x4f::TRS((offsets[tid] * (sensorLinearVelocity * 0.001f)) * 1000.0f, (offsets[tid] * (sensorAngularVelocity) * 0.001f) * 1000.0f, Vec3f(1.0f, 1.0f, 1.0f)) * inRays[tid];
}

__global__ void kTransformPoints(size_t pointCount, const Field<XYZ_F32>::type* inPoints, Field<XYZ_F32>::type* outPoints, Mat3x4f transform)
{
	LIMIT(pointCount);
	outPoints[tid] = transform * inPoints[tid];
}

__global__ void kApplyCompaction(size_t pointCount, size_t fieldSize, const Field<IS_HIT_I32>::type* shouldWrite, const CompactionIndexType*writeIndex, char *dst, const char *src)
{
	LIMIT(pointCount);
	int32_t rIdx = tid;
	if (!shouldWrite[rIdx]) {
		return;
	}
	int wIdx = writeIndex[rIdx] - 1;
	memcpy(dst + fieldSize * wIdx, src + fieldSize * rIdx, fieldSize);
}

__global__ void kCutField(size_t pointCount, char* dst, const char* src, size_t offset, size_t stride, size_t fieldSize)
{
	LIMIT(pointCount);
	memcpy(dst + tid * fieldSize, src + tid * stride + offset, fieldSize);
}

__global__ void kFilter(size_t count, const Field<RAY_IDX_U32>::type* indices, char* dst, char* src, size_t fieldSize)
{
	LIMIT(count);
	memcpy(dst + tid * fieldSize, src + indices[tid] * fieldSize, fieldSize);
}

void gpuFindCompaction(cudaStream_t stream, size_t pointCount, const Field<IS_HIT_I32>::type* isHit, CompactionIndexType* hitCountInclusive, size_t* outHitCount)
{
	// beg and end could be used as const pointers, however thrust does not support it
	auto beg = thrust::device_ptr<const int32_t>(isHit);
	auto end = thrust::device_ptr<const int32_t>(isHit + pointCount);
	auto dst = thrust::device_ptr<int32_t>(hitCountInclusive);

	// Note: this will compile only in a .cu file
	thrust::inclusive_scan(thrust::cuda::par.on(stream), beg, end, dst);
	CHECK_CUDA(cudaMemcpyAsync(outHitCount, hitCountInclusive + pointCount - 1, sizeof(*hitCountInclusive), cudaMemcpyDefault, stream));
}

void gpuFormatSoaToAos(cudaStream_t stream, size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc *soaInData, char *aosOutData)
{ run(kFormatSoaToAos, stream, pointCount, pointSize, fieldCount, soaInData, aosOutData); }

void gpuFormatAosToSoa(cudaStream_t stream, size_t pointCount, size_t pointSize, size_t fieldCount, const char* aosInData, GPUFieldDesc* soaOutData)
{ run(kFormatAosToSoa, stream, pointCount, pointSize, fieldCount, aosInData, soaOutData); }

void gpuTransformRays(cudaStream_t stream, size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{ run(kTransformRays, stream, rayCount, inRays, outRays, transform); };

void gpuVelocityDistortRays(cudaStream_t stream, size_t rayCount, const Mat3x4f* inRays, const float* offsets, size_t offsetsCount, const Vec3f sensorLinearVelocity, const Vec3f sensorAngularVelocity, Mat3x4f* outRays)
{ run(kVelocityDistortRays, stream, rayCount, inRays, offsets, offsetsCount, sensorLinearVelocity, sensorAngularVelocity, outRays); }

void gpuApplyCompaction(cudaStream_t stream, size_t pointCount, size_t fieldSize, const Field<IS_HIT_I32>::type* shouldWrite, const CompactionIndexType *writeIndex, char *dst, const char *src)
{ run(kApplyCompaction, stream, pointCount, fieldSize, shouldWrite, writeIndex, dst, src); }

void gpuTransformPoints(cudaStream_t stream, size_t pointCount, const Field<XYZ_F32>::type* inPoints, Field<XYZ_F32>::type* outPoints, Mat3x4f transform)
{ run(kTransformPoints, stream, pointCount, inPoints, outPoints, transform); }

void gpuCutField(cudaStream_t stream, size_t pointCount, char *dst, const char *src, size_t offset, size_t stride, size_t fieldSize)
{ run(kCutField, stream, pointCount, dst, src, offset, stride, fieldSize); }

void gpuFilter(cudaStream_t stream, size_t count, const Field<RAY_IDX_U32>::type* indices, char *dst, const char *src, size_t fieldSize)
{ run(kFilter, stream, count, indices, dst, src, fieldSize); }
