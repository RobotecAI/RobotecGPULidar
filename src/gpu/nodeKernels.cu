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
#include <thrust/complex.h>

__global__ void kFormatSoaToAos(size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc* soaInData,
                                char* aosOutData)
{
	LIMIT(pointCount);
	for (size_t i = 0; i < fieldCount; ++i) {
		memcpy(aosOutData + pointSize * tid + soaInData[i].dstOffset, soaInData[i].readDataPtr + soaInData[i].size * tid,
		       soaInData[i].size);
	}
}

__global__ void kFormatAosToSoa(size_t pointCount, size_t pointSize, size_t fieldCount, const char* aosInData,
                                const GPUFieldDesc* soaOutData)
{
	LIMIT(pointCount);
	for (size_t i = 0; i < fieldCount; ++i) {
		memcpy(soaOutData[i].writeDataPtr + soaOutData[i].size * tid, aosInData + pointSize * tid + soaOutData[i].dstOffset,
		       soaOutData[i].size);
	}
}

__global__ void kTransformRays(size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{
	LIMIT(rayCount);
	outRays[tid] = transform * inRays[tid];
}

__global__ void kTransformPoints(size_t pointCount, const Field<XYZ_VEC3_F32>::type* inPoints,
                                 Field<XYZ_VEC3_F32>::type* outPoints, Mat3x4f transform)
{
	LIMIT(pointCount);
	outPoints[tid] = transform * inPoints[tid];
}

__global__ void kApplyCompaction(size_t pointCount, size_t fieldSize, const int32_t* shouldWrite,
                                 const CompactionIndexType* writeIndex, char* dst, const char* src)
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

__device__ Vec3f reflectPolarization(const Vec3f& pol, const Vec3f& hitNormal, const Vec3f& rayDir)
{
	const auto diffCrossNormal = rayDir.cross(hitNormal);
	const auto polU = diffCrossNormal.normalized();
	const auto polR = rayDir.cross(polU).normalized();

	const auto refDir = (rayDir - hitNormal * (2 * rayDir.dot(hitNormal))).normalized();
	const auto refPolU = -1.0f * polU;
	const auto refPolR = rayDir.cross(refPolU);

	const auto polCompU = pol.dot(polU);
	const auto polCompR = pol.dot(polR);

	return -polCompR * refPolR + polCompU * refPolU;
}


__global__ void kRadarComputeEnergy(size_t count, float rayAzimuthStepRad, float rayElevationStepRad,
                                    const Field<RAY_POSE_MAT3x4_F32>::type* rayPose, const Field<DISTANCE_F32>::type* hitDist,
                                    const Field<NORMAL_VEC3_F32>::type* hitNorm, const Field<XYZ_VEC3_F32>::type* hitPos,
                                    thrust::complex<float>* outBR, thrust::complex<float>* outBU,
                                    thrust::complex<float>* outFactor)
{
	LIMIT(count);

	constexpr float c0 = 299792458.0f;
	constexpr float freq = 79E9f;
	constexpr float waveLen = c0 / freq;
	constexpr float waveNum = 2.0f * M_PIf / waveLen;
	constexpr float reflectionCoef = 1.0f; // TODO
	const thrust::complex<float> i = {0, 1.0};
	const Vec3f dirX = {1, 0, 0};
	const Vec3f dirY = {0, 1, 0};
	const Vec3f dirZ = {0, 0, 1};

	Vec3f rayDirCts = rayPose[tid] * Vec3f{0, 0, 1};
	Vec3f rayDirSph = {rayDirCts.length(), rayDirCts[0] == 0 && rayDirCts[1] == 0 ? 0 : atan2(rayDirCts.y(), rayDirCts.x()),
	                   std::acos(rayDirCts.z() / rayDirCts.length())};
	float phi = rayDirSph[1]; // azimuth, 0 = X-axis, positive = CCW
	float the = rayDirSph[2]; // elevation, 0 = Z-axis, 90 = XY-plane, -180 = negative Z-axis

	// Consider unit vector of the ray direction, these are its projections:
	float cp = cosf(phi); // X-dir component
	float sp = sinf(phi); // Y-dir component
	float ct = cosf(the); // Z-dir component
	float st = sinf(the); // XY-plane component

	Vec3f dirP = {-sp, cp, 0};
	Vec3f dirT = {cp * ct, sp * ct, -st};

	thrust::complex<float> kr = {waveNum * hitDist[tid], 0.0f};

	Vec3f rayDir = rayDirCts.normalized();
	Vec3f rayPol = rayPose[tid] * Vec3f{-1, 0, 0}; // UP, perpendicular to ray
	Vec3f reflectedPol = reflectPolarization(rayPol, hitNorm[tid], rayDir);

	Vector<3, thrust::complex<float>> rayPolCplx = {reflectedPol.x(), reflectedPol.y(), reflectedPol.z()};

	Vector<3, thrust::complex<float>> apE = reflectionCoef * exp(i * kr) * rayPolCplx;
	Vector<3, thrust::complex<float>> apH = -apE.cross(rayDir);

	Vec3f vecK = waveNum * ((dirX * cp + dirY * sp) * st + dirZ * ct);

	float rayArea = hitDist[tid] * hitDist[tid] * std::sin(rayElevationStepRad) * rayAzimuthStepRad;
	Vector<3, thrust::complex<float>> outBUBRFactor = {0, 0, 0};

	outBUBRFactor[0] = (-(apE.cross(-dirP) + apH.cross(dirT))).dot(rayDir);
	outBUBRFactor[1] = (-(apE.cross(dirT) + apH.cross(dirP))).dot(rayDir);
	outBUBRFactor[2] = thrust::complex<float>(0.0, ((waveNum * rayArea) / (4.0f * M_PIf))) * exp(-i * vecK.dot(hitPos[tid]));
}

__global__ void kFilterGroundPoints(size_t pointCount, const Vec3f sensor_up_vector, float ground_angle_threshold,
                                    const Field<XYZ_VEC3_F32>::type* inPoints, const Field<NORMAL_VEC3_F32>::type* inNormalsPtr,
                                    Field<IS_GROUND_I32>::type* outNonGround, Mat3x4f lidarTransform)
{
	LIMIT(pointCount);

	// Check if normal is pointing up within given threshold.
	const float normalUpAngle = acosf(fabs(inNormalsPtr[tid].dot(sensor_up_vector)));

	outNonGround[tid] = normalUpAngle > ground_angle_threshold;
}


void gpuFindCompaction(cudaStream_t stream, size_t pointCount, const int32_t* shouldCompact,
                       CompactionIndexType* hitCountInclusive, size_t* outHitCount)
{
	// beg and end could be used as const pointers, however thrust does not support it
	auto beg = thrust::device_ptr<const int32_t>(shouldCompact);
	auto end = thrust::device_ptr<const int32_t>(shouldCompact + pointCount);
	auto dst = thrust::device_ptr<int32_t>(hitCountInclusive);

	// Note: this will compile only in a .cu file
	thrust::inclusive_scan(thrust::cuda::par.on(stream), beg, end, dst);
	CHECK_CUDA(cudaMemcpyAsync(outHitCount, hitCountInclusive + pointCount - 1, sizeof(*hitCountInclusive), cudaMemcpyDefault,
	                           stream));
}

void gpuFormatSoaToAos(cudaStream_t stream, size_t pointCount, size_t pointSize, size_t fieldCount,
                       const GPUFieldDesc* soaInData, char* aosOutData)
{
	run(kFormatSoaToAos, stream, pointCount, pointSize, fieldCount, soaInData, aosOutData);
}

void gpuFormatAosToSoa(cudaStream_t stream, size_t pointCount, size_t pointSize, size_t fieldCount, const char* aosInData,
                       const GPUFieldDesc* soaOutData)
{
	run(kFormatAosToSoa, stream, pointCount, pointSize, fieldCount, aosInData, soaOutData);
}

void gpuTransformRays(cudaStream_t stream, size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform)
{
	run(kTransformRays, stream, rayCount, inRays, outRays, transform);
};

void gpuApplyCompaction(cudaStream_t stream, size_t pointCount, size_t fieldSize, const int* shouldWrite,
                        const CompactionIndexType* writeIndex, char* dst, const char* src)
{
	run(kApplyCompaction, stream, pointCount, fieldSize, shouldWrite, writeIndex, dst, src);
}

void gpuTransformPoints(cudaStream_t stream, size_t pointCount, const Field<XYZ_VEC3_F32>::type* inPoints,
                        Field<XYZ_VEC3_F32>::type* outPoints, Mat3x4f transform)
{
	run(kTransformPoints, stream, pointCount, inPoints, outPoints, transform);
}

void gpuCutField(cudaStream_t stream, size_t pointCount, char* dst, const char* src, size_t offset, size_t stride,
                 size_t fieldSize)
{
	run(kCutField, stream, pointCount, dst, src, offset, stride, fieldSize);
}

void gpuFilter(cudaStream_t stream, size_t count, const Field<RAY_IDX_U32>::type* indices, char* dst, const char* src,
               size_t fieldSize)
{
	run(kFilter, stream, count, indices, dst, src, fieldSize);
}

void gpuFilterGroundPoints(cudaStream_t stream, size_t pointCount, const Vec3f sensor_up_vector,
                           float ground_angle_threshold, const Field<XYZ_VEC3_F32>::type* inPoints,
                           const Field<NORMAL_VEC3_F32>::type* inNormalsPtr, Field<IS_GROUND_I32>::type* outNonGround,
                           Mat3x4f lidarTransform)
{
	run(kFilterGroundPoints, stream, pointCount, sensor_up_vector, ground_angle_threshold, inPoints, inNormalsPtr, outNonGround,
	    lidarTransform);
}

void gpuRadarComputeEnergy(cudaStream_t stream, size_t count, float rayAzimuthStepRad, float rayElevationStepRad,
                           const Field<RAY_POSE_MAT3x4_F32>::type* rayPose, const Field<DISTANCE_F32>::type* hitDist,
                           const Field<NORMAL_VEC3_F32>::type* hitNorm, const Field<XYZ_VEC3_F32>::type* hitPos,
                           Vector<3, thrust::complex<float>>* outBUBRFactor)
{
	run(kRadarComputeEnergy, stream, count, rayAzimuthStepRad, rayElevationStepRad, rayPose, hitDist, hitNorm, hitPos,
	    outBUBRFactor);
}
