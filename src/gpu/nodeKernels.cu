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
	const auto refPolR = refDir.cross(refPolU);

	const auto polCompU = pol.dot(polU);
	const auto polCompR = pol.dot(polR);

	return -polCompR * refPolR + polCompU * refPolU;
}

__global__ void kRadarComputeEnergy(size_t count, float rayAzimuthStepRad, float rayElevationStepRad, float freq,
                                    const Field<RAY_POSE_MAT3x4_F32>::type* rayPose, const Field<DISTANCE_F32>::type* hitDist,
                                    const Field<NORMAL_VEC3_F32>::type* hitNorm, const Field<XYZ_VEC3_F32>::type* hitPos,
                                    Vector<3, thrust::complex<float>>* outBUBRFactor)
{
	LIMIT(count);

	constexpr float c0 = 299792458.0f;
	constexpr float reflectionCoef = 1.0f; // TODO
	const float waveLen = c0 / freq;
	const float waveNum = 2.0f * M_PIf / waveLen;
	const thrust::complex<float> i = {0, 1.0};
	const Vec3f dirX = {1, 0, 0};
	const Vec3f dirY = {0, 1, 0};
	const Vec3f dirZ = {0, 0, 1};

	const Vec3f rayDirCts = rayPose[tid] * Vec3f{0, 0, 1};
	const Vec3f rayDirSph = rayDirCts.toSpherical();
	const float phi = rayDirSph[1]; // azimuth, 0 = X-axis, positive = CCW
	const float the = rayDirSph[2]; // elevation, 0 = Z-axis, 90 = XY-plane, -180 = negative Z-axis

	// Consider unit vector of the ray direction, these are its projections:
	const float cp = cosf(phi); // X-dir component
	const float sp = sinf(phi); // Y-dir component
	const float ct = cosf(the); // Z-dir component
	const float st = sinf(the); // XY-plane component

	const Vec3f dirP = {-sp, cp, 0};
	const Vec3f dirT = {cp * ct, sp * ct, -st};

	const float kr = waveNum * hitDist[tid];

	const Vec3f rayDir = rayDirCts.normalized();
	const Vec3f rayPol = rayPose[tid] * Vec3f{-1, 0, 0}; // UP, perpendicular to ray
	const Vec3f reflectedPol = reflectPolarization(rayPol, hitNorm[tid], rayDir);

	const Vector<3, thrust::complex<float>> rayPolCplx = {reflectedPol.x(), reflectedPol.y(), reflectedPol.z()};

	const Vector<3, thrust::complex<float>> apE = reflectionCoef * exp(i * kr) * rayPolCplx;
	const Vector<3, thrust::complex<float>> apH = -apE.cross(rayDir);

	const Vec3f vecK = waveNum * ((dirX * cp + dirY * sp) * st + dirZ * ct);

	const float rayArea = hitDist[tid] * hitDist[tid] * std::sin(rayElevationStepRad) * rayAzimuthStepRad;

	thrust::complex<float> BU = (-(apE.cross(-dirP) + apH.cross(dirT))).dot(rayDir);
	thrust::complex<float> BR = (-(apE.cross(dirT) + apH.cross(dirP))).dot(rayDir);
	thrust::complex<float> factor = thrust::complex<float>(0.0, ((waveNum * rayArea) / (4.0f * M_PIf))) *
	                                exp(-i * vecK.dot(hitPos[tid]));

	//	printf("GPU: point=%d ray=??: dist=%f, pos=(%.2f, %.2f, %.2f), norm=(%.2f, %.2f, %.2f), BU=(%.2f+%.2fi), BR=(%.2f+%.2fi), factor=(%.2f+%.2fi)\n", tid, hitDist[tid],
	//	       hitPos[tid].x(), hitPos[tid].y(), hitPos[tid].z(), hitNorm[tid].x(), hitNorm[tid].y(), hitNorm[tid].z(),
	//	       BU.real(), BU.imag(), BR.real(), BR.imag(), factor.real(), factor.imag());
	// Print variables:
	//	printf("GPU: point=%d ray=??: rayDirCts=(%.2f, %.2f, %.2f), rayDirSph=(%.2f, %.2f, %.2f), phi=%.2f, the=%.2f, cp=%.2f, "
	//	       "sp=%.2f, ct=%.2f, st=%.2f, dirP=(%.2f, %.2f, %.2f), dirT=(%.2f, %.2f, %.2f), kr=(%.2f+%.2fi), rayDir=(%.2f, "
	//	       "%.2f, %.2f), rayPol=(%.2f, %.2f, %.2f), reflectedPol=(%.2f, %.2f, %.2f)\n",
	//	       tid, rayDirCts.x(), rayDirCts.y(), rayDirCts.z(), rayDirSph.x(), rayDirSph.y(),
	//	       rayDirSph.z(), phi, the, cp, sp, ct, st, dirP.x(), dirP.y(), dirP.z(), dirT.x(), dirT.y(), dirT.z(), kr.real(),
	//	       kr.imag(), rayDir.x(), rayDir.y(), rayDir.z(), rayPol.x(), rayPol.y(), rayPol.z(), reflectedPol.x(),
	//	       reflectedPol.y(), reflectedPol.z());

	outBUBRFactor[tid] = {BU, BR, factor};
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

void gpuRadarComputeEnergy(cudaStream_t stream, size_t count, float rayAzimuthStepRad, float rayElevationStepRad, float freq,
                           const Field<RAY_POSE_MAT3x4_F32>::type* rayPose, const Field<DISTANCE_F32>::type* hitDist,
                           const Field<NORMAL_VEC3_F32>::type* hitNorm, const Field<XYZ_VEC3_F32>::type* hitPos,
                           Vector<3, thrust::complex<float>>* outBUBRFactor)
{
	run(kRadarComputeEnergy, stream, count, rayAzimuthStepRad, rayElevationStepRad, freq, rayPose, hitDist, hitNorm, hitPos,
	    outBUBRFactor);
}
