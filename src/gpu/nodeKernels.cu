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
	if (abs(rayDir.dot(hitNormal)) == 1) {
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

__device__ void saveReturnHit(const RaytraceRequestContext* ctx, int beamIdx, int sampleIdx, int returnPointIdx, int returnType)
{
	if (ctx->xyz != nullptr) {
		const Mat3x4f ray = ctx->raysWorld[beamIdx];
		const Vec3f origin = ray * Vec3f{0, 0, 0};
		const Vec3f dir = (ray * Vec3f{0, 0, 1} - origin).normalized();
		ctx->xyz[returnPointIdx] = origin + dir * ctx->mrSamples.distance[sampleIdx];
	}
	if (ctx->isHit != nullptr) {
		ctx->isHit[returnPointIdx] = ctx->mrSamples.isHit[sampleIdx];
	}
	if (ctx->ringIdx != nullptr && ctx->ringIds != nullptr) {
		ctx->ringIdx[returnPointIdx] = ctx->ringIds[beamIdx % ctx->ringIdsCount];
	}
	if (ctx->returnType != nullptr) {
		ctx->returnType[returnPointIdx] = returnType;
	}
	if (ctx->distance != nullptr) {
		ctx->distance[returnPointIdx] = ctx->mrSamples.distance[sampleIdx];
	}
	if (ctx->intensityF32 != nullptr) {
		ctx->intensityF32[returnPointIdx] = ctx->mrSamples.intensity[sampleIdx];
	}
	if (ctx->intensityU8 != nullptr) {
		// intensity < 0 not possible
		ctx->intensityU8[returnPointIdx] = ctx->mrSamples.intensity[sampleIdx] < UINT8_MAX ?
		                                       static_cast<uint8_t>(std::round(ctx->mrSamples.intensity[sampleIdx])) :
		                                       UINT8_MAX;
	}
	if (ctx->entityId != nullptr) {
		ctx->entityId[returnPointIdx] = ctx->mrSamples.entityId[sampleIdx];
	}
	if (ctx->pointAbsVelocity != nullptr) {
		ctx->pointAbsVelocity[returnPointIdx] = ctx->mrSamples.absVelocity[sampleIdx];
	}
	if (ctx->pointRelVelocity != nullptr) {
		ctx->pointRelVelocity[returnPointIdx] = ctx->mrSamples.relVelocity[sampleIdx];
	}
	if (ctx->radialSpeed != nullptr) {
		ctx->radialSpeed[returnPointIdx] = ctx->mrSamples.radialSpeed[sampleIdx];
	}
	if (ctx->normal != nullptr) {
		ctx->normal[returnPointIdx] = ctx->mrSamples.normal[sampleIdx];
	}
	if (ctx->incidentAngle != nullptr) {
		ctx->incidentAngle[returnPointIdx] = ctx->mrSamples.incidentAngle[sampleIdx];
	}
	if (ctx->laserRetro != nullptr) {
		ctx->laserRetro[returnPointIdx] = ctx->mrSamples.laserRetro[sampleIdx];
	}
}

__device__ void saveReturnNonHit(const RaytraceRequestContext* ctx, int beamIdx, int returnPointIdx, int returnType)
{
	// Arbitrary decision - if all samples are non hit, I just take the distance from center ray. This distance will be either
	// ctx.nearNonHitDistance or ctx.farNonHitDistance, based on optix kernel processing.
	const auto nonHitDistance = ctx->mrSamples.distance[0];

	if (ctx->xyz != nullptr) {
		const Mat3x4f ray = ctx->raysWorld[beamIdx];
		const Vec3f origin = ray * Vec3f{0, 0, 0};
		const Vec3f dir = ray * Vec3f{0, 0, 1} - origin;
		Vec3f displacement = dir.normalized() * nonHitDistance;
		displacement = {isnan(displacement.x()) ? 0 : displacement.x(), isnan(displacement.y()) ? 0 : displacement.y(),
		                isnan(displacement.z()) ? 0 : displacement.z()};
		ctx->xyz[returnPointIdx] = origin + displacement;
	}
	if (ctx->isHit != nullptr) {
		ctx->isHit[returnPointIdx] = false;
	}
	if (ctx->ringIdx != nullptr && ctx->ringIds != nullptr) {
		ctx->ringIdx[returnPointIdx] = ctx->ringIds[beamIdx % ctx->ringIdsCount];
	}
	if (ctx->returnType != nullptr) {
		ctx->returnType[returnPointIdx] = returnType;
	}
	if (ctx->distance != nullptr) {
		ctx->distance[returnPointIdx] = nonHitDistance;
	}
	if (ctx->intensityF32 != nullptr) {
		ctx->intensityF32[returnPointIdx] = 0;
	}
	if (ctx->intensityU8 != nullptr) {
		// intensity < 0 not possible
		ctx->intensityU8[returnPointIdx] = 0;
	}
	if (ctx->entityId != nullptr) {
		ctx->entityId[returnPointIdx] = RGL_ENTITY_INVALID_ID;
	}
	if (ctx->pointAbsVelocity != nullptr) {
		ctx->pointAbsVelocity[returnPointIdx] = Vec3f{NAN};
	}
	if (ctx->pointRelVelocity != nullptr) {
		ctx->pointRelVelocity[returnPointIdx] = Vec3f{NAN};
	}
	if (ctx->radialSpeed != nullptr) {
		ctx->radialSpeed[returnPointIdx] = 0;
	}
	if (ctx->normal != nullptr) {
		ctx->normal[returnPointIdx] = Vec3f{NAN};
	}
	if (ctx->incidentAngle != nullptr) {
		ctx->incidentAngle[returnPointIdx] = NAN;
	}
	if (ctx->laserRetro != nullptr) {
		ctx->laserRetro[returnPointIdx] = 0;
	}
}

__global__ void kRadarComputeEnergy(size_t count, float rayAzimuthStepRad, float rayElevationStepRad, float freq,
                                    Mat3x4f lookAtOriginTransform, const Field<RAY_POSE_MAT3x4_F32>::type* rayPose,
                                    const Field<DISTANCE_F32>::type* hitDist, const Field<NORMAL_VEC3_F32>::type* hitNorm,
                                    const Field<XYZ_VEC3_F32>::type* hitPos, Vector<3, thrust::complex<float>>* outBUBRFactor)
{
	LIMIT(count);

	constexpr bool log = false;

	constexpr float c0 = 299792458.0f;
	constexpr float reflectionCoef = 1.0f; // TODO
	const float waveLen = c0 / freq;
	const float waveNum = 2.0f * static_cast<float>(M_PI) / waveLen;
	const thrust::complex<float> i = {0, 1.0};
	const Vec3f dirX = {1, 0, 0};
	const Vec3f dirY = {0, 1, 0};
	const Vec3f dirZ = {0, 0, 1};

	const Mat3x4f rayPoseLocal = lookAtOriginTransform * rayPose[tid];
	//	const Vec3f hitPosLocal = lookAtOriginTransform * hitPos[tid];
	const Vec3f rayDir = rayPoseLocal * Vec3f{0, 0, 1};
	const Vec3f rayPol = rayPoseLocal * Vec3f{1, 0, 0}; // UP, perpendicular to ray
	const Vec3f hitNormalLocal = lookAtOriginTransform.rotation() * hitNorm[tid];
	const float hitDistance = hitDist[tid];
	const float rayArea = hitDistance * hitDistance * sinf(rayElevationStepRad) * rayAzimuthStepRad;

	if (log)
		printf("rayDir: (%.4f %.4f %.4f) rayPol: (%.4f %.4f %.4f) hitNormal: (%.4f %.4f %.4f)\n", rayDir.x(), rayDir.y(),
		       rayDir.z(), rayPol.x(), rayPol.y(), rayPol.z(), hitNormalLocal.x(), hitNormalLocal.y(), hitNormalLocal.z());

	const float phi = atan2(rayDir[1], rayDir[2]);
	const float the = acos(rayDir[0] / rayDir.length());

	// Consider unit vector of the ray direction, these are its projections:
	const float cp = cosf(phi); // X-dir component
	const float sp = sinf(phi); // Y-dir component
	const float ct = cosf(the); // Z-dir component
	const float st = sinf(the); // XY-plane component

	const Vec3f dirP = {0, cp, -sp};
	const Vec3f dirT = {-st, sp * ct, cp * ct};
	const Vec3f vecK = waveNum * ((dirZ * cp + dirY * sp) * st + dirX * ct);

	if (log)
		printf("phi: %.2f [dirP: (%.4f %.4f %.4f)] theta: %.2f [dirT: (%.4f %.4f %.4f)] vecK=(%.2f, %.2f, %.2f)\n", phi,
		       dirP.x(), dirP.y(), dirP.z(), the, dirT.x(), dirT.y(), dirT.z(), vecK.x(), vecK.y(), vecK.z());

	const Vec3f reflectedDir = (rayDir - hitNormalLocal * (2 * rayDir.dot(hitNormalLocal))).normalized();
	const Vec3f reflectedPol = reflectPolarization(rayPol, hitNormalLocal, rayDir);
	const Vector<3, thrust::complex<float>> reflectedPolCplx = {reflectedPol.x(), reflectedPol.y(), reflectedPol.z()};
	const float kr = waveNum * hitDistance;

	if (log)
		printf("reflectedDir: (%.4f %.4f %.4f) reflectedPol: (%.4f %.4f %.4f)\n", reflectedDir.x(), reflectedDir.y(),
		       reflectedDir.z(), reflectedPol.x(), reflectedPol.y(), reflectedPol.z());

	const Vector<3, thrust::complex<float>> apE = reflectionCoef * exp(i * kr) * reflectedPolCplx;
	const Vector<3, thrust::complex<float>> apH = -apE.cross(reflectedDir);

	if (log)
		printf("apE: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n", apE.x().real(), apE.x().imag(), apE.y().real(),
		       apE.y().imag(), apE.z().real(), apE.z().imag());
	if (log)
		printf("apH: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n", apH.x().real(), apH.x().imag(), apH.y().real(),
		       apH.y().imag(), apH.z().real(), apH.z().imag());

	const Vector<3, thrust::complex<float>> BU1 = apE.cross(-dirP);
	const Vector<3, thrust::complex<float>> BU2 = apH.cross(dirT);
	const Vector<3, thrust::complex<float>> refField1 = (-(BU1 + BU2));

	if (log)
		printf("BU1: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		       "BU2: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		       "refField1: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n",
		       BU1.x().real(), BU1.x().imag(), BU1.y().real(), BU1.y().imag(), BU1.z().real(), BU1.z().imag(), BU2.x().real(),
		       BU2.x().imag(), BU2.y().real(), BU2.y().imag(), BU2.z().real(), BU2.z().imag(), refField1.x().real(),
		       refField1.x().imag(), refField1.y().real(), refField1.y().imag(), refField1.z().real(), refField1.z().imag());

	const Vector<3, thrust::complex<float>> BR1 = apE.cross(dirT);
	const Vector<3, thrust::complex<float>> BR2 = apH.cross(dirP);
	const Vector<3, thrust::complex<float>> refField2 = (-(BR1 + BR2));

	if (log)
		printf("BR1: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		       "BR2: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n"
		       "refField2: [(%.2f + %.2fi) (%.2f + %.2fi) (%.2f + %.2fi)]\n",
		       BR1.x().real(), BR1.x().imag(), BR1.y().real(), BR1.y().imag(), BR1.z().real(), BR1.z().imag(), BR2.x().real(),
		       BR2.x().imag(), BR2.y().real(), BR2.y().imag(), BR2.z().real(), BR2.z().imag(), refField2.x().real(),
		       refField2.x().imag(), refField2.y().real(), refField2.y().imag(), refField2.z().real(), refField2.z().imag());

	const thrust::complex<float> BU = refField1.dot(reflectedDir);
	const thrust::complex<float> BR = refField2.dot(reflectedDir);
	//	const thrust::complex<float> factor = thrust::complex<float>(0.0, ((waveNum * rayArea) / (4.0f * static_cast<float>(M_PI)))) *
	//	                                      exp(-i * waveNum * hitDistance);
	const thrust::complex<float> factor = thrust::complex<float>(0.0, ((waveNum * rayArea * reflectedDir.dot(hitNormalLocal)) /
	                                                                   (4.0f * static_cast<float>(M_PI)))) *
	                                      exp(-i * waveNum * hitDistance);
	//	const thrust::complex<float> factor = thrust::complex<float>(0.0, ((waveNum * rayArea) / (4.0f * static_cast<float>(M_PI)))) *
	//	                                      exp(-i * vecK.dot(hitPosLocal));

	const auto BUf = BU * factor;
	const auto BRf = BR * factor;

	if (log)
		printf("BU: (%.2f + %.2fi) BR: (%.2f + %.2fi) factor: (%.2f + %.2fi) [BUf: (%.2f + %.2fi) BRf: %.2f + %.2fi]\n",
		       BU.real(), BU.imag(), BR.real(), BR.imag(), factor.real(), factor.imag(), BUf.real(), BUf.imag(), BRf.real(),
		       BRf.imag());

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

__global__ void kProcessBeamSamplesFirstLast(size_t beamCount, int samplesPerBeam, MultiReturnPointers beamSamples,
                                             MultiReturnPointers first, MultiReturnPointers last, const Mat3x4f* beamsWorld)
{
	LIMIT(beamCount);

	const auto beamIdx = tid;
	int firstIdx = -1;
	int lastIdx = -1;
	for (int sampleIdx = 0; sampleIdx < samplesPerBeam; ++sampleIdx) {
		if (beamSamples.isHit[beamIdx * samplesPerBeam + sampleIdx] == 0) {
			continue;
		}
		auto currentFirstDistance = firstIdx >= 0 ? beamSamples.distance[beamIdx * samplesPerBeam + firstIdx] : FLT_MAX;
		auto currentLastDistance = lastIdx >= 0 ? beamSamples.distance[beamIdx * samplesPerBeam + lastIdx] : -FLT_MAX;
		if (beamSamples.distance[beamIdx * samplesPerBeam + sampleIdx] < currentFirstDistance) {
			firstIdx = sampleIdx;
		}
		if (beamSamples.distance[beamIdx * samplesPerBeam + sampleIdx] > currentLastDistance) {
			lastIdx = sampleIdx;
		}
	}
	Vec3f beamOrigin = beamsWorld[beamIdx] * Vec3f{0, 0, 0};
	Vec3f beamDir = ((beamsWorld[beamIdx] * Vec3f{0, 0, 1}) - beamOrigin).normalized();
	bool isHit = firstIdx >= 0; // Note that firstHit >= 0 implies lastHit >= 0
	first.isHit[beamIdx] = isHit;
	last.isHit[beamIdx] = isHit;
	if (isHit) {
		first.distance[beamIdx] = beamSamples.distance[beamIdx * samplesPerBeam + firstIdx];
		last.distance[beamIdx] = beamSamples.distance[beamIdx * samplesPerBeam + lastIdx];
		first.xyz[beamIdx] = beamOrigin + beamDir * first.distance[beamIdx];
		last.xyz[beamIdx] = beamOrigin + beamDir * last.distance[beamIdx];
	}
}

__global__ void kReduceDivergentBeams(size_t beamCount, int samplesPerBeam, rgl_return_mode_t returnMode,
                                      const RaytraceRequestContext* ctx)
{
	LIMIT(beamCount);

	const auto beamIdx = tid;
	const auto firstSampleInBeamIdx = beamIdx * samplesPerBeam;
	int first = -1, second = -1, last = -1;
	int strongest = -1, secondStrongest = -1;
	int sampleIdx = firstSampleInBeamIdx;

//	printf("firstSampleIdx: %d samplesPerBeam: %d\n", firstSampleInBeamIdx, samplesPerBeam);

	for (; sampleIdx < firstSampleInBeamIdx + samplesPerBeam; ++sampleIdx) {
		//		printf("sample after optix %f %f %f\n", ctx->mrSamples.xyz[sampleIdx].x(), ctx->mrSamples.xyz[sampleIdx].y(),
		//		       ctx->mrSamples.xyz[sampleIdx].z());
		if (ctx->mrSamples.isHit[sampleIdx] == 1) {
			first = sampleIdx;
			second = first;
			last = first;
			strongest = sampleIdx;
			secondStrongest = strongest;
			break;
		}
	}

	const auto returnCount = returnMode >> RGL_RETURN_MODE_BIT_SHIFT;
	//printf("returnMode: %d returnCount: %d\n", returnMode, returnCount);

	// There were no hits within beam samples.
	if (first == -1) {
		for (int returnIdx = 0; returnIdx < returnCount; ++returnIdx) {
			const auto returnPointIdx = beamIdx * returnCount + returnIdx;
			const auto returnType = (returnMode >> (returnIdx * RGL_RETURN_TYPE_BIT_SHIFT)) & 0xff;
			saveReturnNonHit(ctx, beamIdx, returnPointIdx, returnType);
		}
		return;
	}

	//printf("reference sample idx: %d\n", sampleIdx);

	// Base initial values on first sample that returned a hit.
	for (sampleIdx = sampleIdx + 1; sampleIdx < firstSampleInBeamIdx + samplesPerBeam; ++sampleIdx) {
		if (ctx->mrSamples.isHit[sampleIdx] == 0) {
			continue;
		}

		// TODO(Pawel): Revise this logic.
		const auto currentSampleDistance = ctx->mrSamples.distance[sampleIdx];
		if (currentSampleDistance < ctx->mrSamples.distance[first]) {
			// Sample is closer than first. I ignore current == first, because then second would become equal to first.
			second = first;
			first = sampleIdx;
		} else if (ctx->mrSamples.distance[first] < currentSampleDistance &&
		           currentSampleDistance < ctx->mrSamples.distance[second]) {
			// Sample is farther than first and closer than second. Note that current == first distance is ignored here -
			// otherwise second would become first everytime when current would be equal first.
			second = sampleIdx;
		}

		if (currentSampleDistance > ctx->mrSamples.distance[last]) {
			last = sampleIdx;
		}

		const auto currentIntensity = ctx->mrSamples.intensity[sampleIdx];
		if (currentIntensity > ctx->mrSamples.intensity[strongest]) {
			secondStrongest = strongest;
			strongest = sampleIdx;
		} else if (ctx->mrSamples.intensity[strongest] > currentIntensity &&
		           currentIntensity > ctx->mrSamples.intensity[secondStrongest]) {
			secondStrongest = sampleIdx;
		}
	}

//		printf("first: %d, second: %d, last: %d, strongest: %d, secondStrongest: %d\n", first, second, last, strongest,
//		       secondStrongest);

	for (int returnIdx = 0; returnIdx < returnCount; ++returnIdx) {
		const auto returnType = (returnMode >> (returnIdx * RGL_RETURN_TYPE_BIT_SHIFT)) & 0xff;
		int rayIdx = -1;

//		printf("return idx: %d test: %d return types: %d\n", returnIdx, (returnMode >> (returnIdx * RGL_RETURN_TYPE_BIT_SHIFT)) & 0xff, returnType);

		if (returnType == RGL_RETURN_TYPE_FIRST) {
			rayIdx = first;
		} else if (returnType == RGL_RETURN_TYPE_SECOND) {
			rayIdx = second;
		} else if (returnType == RGL_RETURN_TYPE_LAST) {
			rayIdx = last;
		} else if (returnType == RGL_RETURN_TYPE_STRONGEST) {
			rayIdx = strongest;
		} else if (returnType == RGL_RETURN_TYPE_SECOND_STRONGEST) {
			rayIdx = secondStrongest;
		}

		if (rayIdx >= 0) {
			const auto returnPointIdx = beamIdx * returnCount + returnIdx;
//			printf("assigned sample: %f %f %f\n", ctx->mrSamples.xyz[rayIdx].x(), ctx->mrSamples.xyz[rayIdx].y(),
//			       ctx->mrSamples.xyz[rayIdx].z());
			saveReturnHit(ctx, beamIdx, rayIdx, returnPointIdx, returnType);
			//			printf("assigned xyz: %f %f %f\n", ctx->xyz[returnPointIdx].x(), ctx->xyz[returnPointIdx].y(),
			//			       ctx->xyz[returnPointIdx].z());
			//printf("%f %f %f\n", ctx->mrSamples.xyz[sampleIdx].x(), ctx->mrSamples.xyz[sampleIdx].y(), ctx->mrSamples.xyz[sampleIdx].z());
		}
	}
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

void gpuFilterGroundPoints(cudaStream_t stream, size_t pointCount, const Vec3f sensor_up_vector, float ground_angle_threshold,
                           const Field<XYZ_VEC3_F32>::type* inPoints, const Field<NORMAL_VEC3_F32>::type* inNormalsPtr,
                           Field<IS_GROUND_I32>::type* outNonGround, Mat3x4f lidarTransform)
{
	run(kFilterGroundPoints, stream, pointCount, sensor_up_vector, ground_angle_threshold, inPoints, inNormalsPtr, outNonGround,
	    lidarTransform);
}

void gpuRadarComputeEnergy(cudaStream_t stream, size_t count, float rayAzimuthStepRad, float rayElevationStepRad, float freq,
                           Mat3x4f lookAtOriginTransform, const Field<RAY_POSE_MAT3x4_F32>::type* rayPose,
                           const Field<DISTANCE_F32>::type* hitDist, const Field<NORMAL_VEC3_F32>::type* hitNorm,
                           const Field<XYZ_VEC3_F32>::type* hitPos, Vector<3, thrust::complex<float>>* outBUBRFactor)
{
	run(kRadarComputeEnergy, stream, count, rayAzimuthStepRad, rayElevationStepRad, freq, lookAtOriginTransform, rayPose,
	    hitDist, hitNorm, hitPos, outBUBRFactor);
}

void gpuProcessBeamSamplesFirstLast(cudaStream_t stream, size_t beamCount, int samplesPerBeam, MultiReturnPointers beamSamples,
                                    MultiReturnPointers first, MultiReturnPointers last, const Mat3x4f* beamsWorld)
{
	run(kProcessBeamSamplesFirstLast, stream, beamCount, samplesPerBeam, beamSamples, first, last, beamsWorld);
}

void gpuReduceDivergentBeams(cudaStream_t stream, size_t beamCount, int samplesPerBeam, rgl_return_mode_t returnMode,
                             const RaytraceRequestContext* ctx)
{
	run(kReduceDivergentBeams, stream, beamCount, samplesPerBeam, returnMode, ctx);
}
