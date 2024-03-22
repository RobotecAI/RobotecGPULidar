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
	// Normal incidence
	if (abs(rayDir.dot(hitNormal)) == 1)
	{
		return -pol;
	}

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

	bool log = false;

	const Vec3f rayDir = (rayPose[tid] * Vec3f{0, 0, 1}).normalized();
	const Vec3f rayPol = rayPose[tid] * Vec3f{1, 0, 0}; // UP, perpendicular to ray
	const Vec3f hitNormal = hitNorm[tid];
	const float hitDistance = hitDist[tid];
	const float rayArea = hitDistance * hitDistance * sinf(rayElevationStepRad) * rayAzimuthStepRad;

	if (log) printf("rayDir: (%.4f %.4f %.4f) rayPol: (%.4f %.4f %.4f) hitNormal: (%.4f %.4f %.4f)\n",
		    rayDir.x(), rayDir.y(), rayDir.z(), rayPol.x(), rayPol.y(), rayPol.z(), hitNormal.x(), hitNormal.y(), hitNormal.z());

	const float phi = atan2(rayDir[1], rayDir[2]);
	const float the = acos(rayDir[0] / rayDir.length());

	// Consider unit vector of the ray direction, these are its projections:
	const float cp = cosf(phi); // X-dir component
	const float sp = sinf(phi); // Y-dir component
	const float ct = cosf(the); // Z-dir component
	const float st = sinf(the); // XY-plane component

	const Vec3f dirP = { 0, cp, -sp };
	const Vec3f dirT = { -st, sp * ct, cp * ct };
	const Vec3f vecK = waveNum * ((dirZ * cp + dirY * sp) * st + dirX * ct);

	if (log) printf("phi: %.2f [dirP: (%.4f %.4f %.4f)] theta: %.2f [dirT: (%.4f %.4f %.4f)] vecK=(%.2f, %.2f, %.2f)\n",
		       phi, dirP.x(), dirP.y(), dirP.z(), the, dirT.x(), dirT.y(), dirT.z(), vecK.x(), vecK.y(), vecK.z());

	const Vec3f reflectedDir = (rayDir - hitNormal * (2 * rayDir.dot(hitNormal))).normalized();
	const Vec3f reflectedPol = reflectPolarization(rayPol, hitNormal, rayDir);
	const Vector<3, thrust::complex<float>> reflectedPolCplx = { reflectedPol.x(), reflectedPol.y(), reflectedPol.z() };
	const float kr = waveNum * hitDist[tid];

	if (log) printf("reflectedDir: (%.4f %.4f %.4f) reflectedPol: (%.4f %.4f %.4f)\n",
		       reflectedDir.x(), reflectedDir.y(), reflectedDir.z(), reflectedPol.x(), reflectedPol.y(), reflectedPol.z());

	const Vector<3, thrust::complex<float>> apE = reflectionCoef * exp(i * kr) * reflectedPolCplx;
	const Vector<3, thrust::complex<float>> apH = -apE.cross(reflectedDir);

	if (log) printf("apE: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n",
		       apE.x().real(), apE.x().imag(), apE.y().real(), apE.y().imag(), apE.z().real(), apE.z().imag());
	if (log) printf("apH: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n",
		       apH.x().real(), apH.x().imag(), apH.y().real(), apH.y().imag(), apH.z().real(), apH.z().imag());

	const Vector<3, thrust::complex<float>> BU1 = apE.cross(-dirP);
	const Vector<3, thrust::complex<float>> BU2 = apH.cross(dirT);
	const Vector<3, thrust::complex<float>> refField1 = (-(BU1 + BU2));

	if (log) printf(
		    "BU1: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		    "BU2: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		    "refField1: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n",
		    BU1.x().real(), BU1.x().imag(), BU1.y().real(), BU1.y().imag(), BU1.z().real(), BU1.z().imag(),
		    BU2.x().real(), BU2.x().imag(), BU2.y().real(), BU2.y().imag(), BU2.z().real(), BU2.z().imag(),
		    refField1.x().real(), refField1.x().imag(), refField1.y().real(), refField1.y().imag(), refField1.z().real(), refField1.z().imag());

	const Vector<3, thrust::complex<float>> BR1 = apE.cross(dirT);
	const Vector<3, thrust::complex<float>> BR2 = apH.cross(dirP);
	const Vector<3, thrust::complex<float>> refField2 = (-(BR1 + BR2));

	if (log) printf(
		    "BR1: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		    "BR2: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		    "refField2: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n",
		    BR1.x().real(), BR1.x().imag(), BR1.y().real(), BR1.y().imag(), BR1.z().real(), BR1.z().imag(),
		    BR2.x().real(), BR2.x().imag(), BR2.y().real(), BR2.y().imag(), BR2.z().real(), BR2.z().imag(),
		    refField2.x().real(), refField2.x().imag(), refField2.y().real(), refField2.y().imag(), refField2.z().real(), refField2.z().imag());

	const thrust::complex<float> BU = refField1.dot(reflectedDir);
	const thrust::complex<float> BR = refField2.dot(reflectedDir);
	const thrust::complex<float> factor = thrust::complex<float>(0.0, ((waveNum * rayArea) / (4.0f * M_PIf))) *
	                                exp(-i * waveNum * hitDistance);

	const auto BUf = BU * factor;
	const auto BRf = BR * factor;

	if (log) printf("BU: (%.2f + %.2fi) BR: (%.2f + %.2fi) factor: (%.2f + %.2fi) [BUf: (%.2f + %.2fi) BRf: %.2f + %.2fi]\n",
		       BU.real(), BU.imag(), BR.real(), BR.imag(), factor.real(), factor.imag(),
		       BUf.real(), BUf.imag(), BRf.real(), BRf.imag());

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
