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

#include <cuda_runtime.h>
#include <math_constants.h>
#include <optix_device.h>
#include <cuda_fp16.h>

#include <math/Vector.hpp>
#include <math/Mat3x4f.hpp>
#include <cassert>

#include <gpu/RaytraceRequestContext.hpp>
#include <gpu/ShaderBindingTableTypes.h>

#include <rgl/api/core.h>

// Constants
static constexpr float toDeg = (180.0f / M_PI);

// Globals
extern "C" static __constant__ RaytraceRequestContext ctx;

// Helper functions
__device__ void saveSampleAsNonHit(int sampleIdx, float nonHitDistance);
__device__ void saveSampleAsHit(int sampleIdx, float distance, float intensity, float laserRetro, int objectID,
                                const Vec3f& absVelocity, const Vec3f& relVelocity, float radialSpeed, const Vec3f& normal,
                                float incidentAngle);
__device__ void saveNonHitBeamSamples(int beamIdx, float nonHitDistance);
__device__ void saveBeamSharedData(int beamIdx, const Mat3x4f& rayLocal);
__device__ void shootSamplingRay(const Mat3x4f& ray, float maxRange, unsigned sampleBeamIdx);
__device__ Mat3x4f makeBeamSampleRayTransform(float hHalfDivergenceRad, float vHalfDivergenceRad, unsigned layerIdx,
                                              unsigned vertexIdx);

extern "C" __global__ void __raygen__()
{
	const int rayIdx = static_cast<int>(optixGetLaunchIndex().x);
	Mat3x4f ray = ctx.raysWorld[rayIdx];
	const Mat3x4f rayLocal =
	    ctx.rayOriginToWorld.inverse() *
	    ray; // TODO(prybicki): instead of computing inverse, we should pass rays in local CF and then transform them to world CF.

	// Saving data for non-hit samples is necessary here - otherwise this data will not be initialized.
	saveNonHitBeamSamples(rayIdx, ctx.farNonHitDistance);
	saveBeamSharedData(rayIdx, rayLocal);
	if (ctx.rayMask != nullptr && ctx.rayMask[rayIdx] == 0) {
		return;
	}

	if (ctx.doApplyDistortion) {
		// Velocities are in the local frame. Need to operate on rays in local frame.
		// Ray time offsets are in milliseconds, velocities are in unit per seconds.
		// In order to not lose numerical precision, first multiply values and then convert to proper unit.
		ray = Mat3x4f::TRS((ctx.rayTimeOffsetsMs[rayIdx] * ctx.sensorLinearVelocityXYZ) * 0.001f,
		                   (ctx.rayTimeOffsetsMs[rayIdx] * (ctx.sensorAngularVelocityRPY * toDeg)) * 0.001f) *
		      rayLocal;
		// Back to the global frame.
		ray = ctx.rayOriginToWorld * ray;
	}

	float maxRange = ctx.rayRangesCount == 1 ? ctx.rayRanges[0].y() : ctx.rayRanges[rayIdx].y();

	shootSamplingRay(ray, maxRange, 0); // Shoot primary ray
	if (ctx.hBeamHalfDivergenceRad > 0.0f && ctx.vBeamHalfDivergenceRad > 0.0f) {
		// Shoot multi-return sampling rays
		for (int layerIdx = 0; layerIdx < MULTI_RETURN_BEAM_LAYERS; ++layerIdx) {
			for (int vertexIdx = 0; vertexIdx < MULTI_RETURN_BEAM_VERTICES; vertexIdx++) {
				Mat3x4f sampleRay = ray * makeBeamSampleRayTransform(ctx.hBeamHalfDivergenceRad, ctx.vBeamHalfDivergenceRad,
				                                                     layerIdx, vertexIdx);
				// Sampling rays indexes start from 1, 0 is reserved for the primary ray
				const unsigned beamSampleRayIdx = 1 + layerIdx * MULTI_RETURN_BEAM_VERTICES + vertexIdx;
				shootSamplingRay(sampleRay, maxRange, beamSampleRayIdx);
			}
		}
	}
}

extern "C" __global__ void __miss__()
{
	const int beamIdx = static_cast<int>(optixGetLaunchIndex().x);
	const int beamSampleIdx = static_cast<int>(optixGetPayload_0());
	saveSampleAsNonHit(beamIdx * MULTI_RETURN_BEAM_SAMPLES + beamSampleIdx, ctx.farNonHitDistance);
}

extern "C" __global__ void __closesthit__()
{
	const EntitySBTData& entityData = *(reinterpret_cast<const EntitySBTData*>(optixGetSbtDataPointer()));

	// Triangle
	const int primID = static_cast<int>(optixGetPrimitiveIndex());
	assert(primID < entityData.indexCount);
	const Vec3i triangleIndices = entityData.index[primID];
	const float u = optixGetTriangleBarycentrics().x;
	const float v = optixGetTriangleBarycentrics().y;
	assert(triangleIndices.x() < entityData.vertexCount);
	assert(triangleIndices.y() < entityData.vertexCount);
	assert(triangleIndices.z() < entityData.vertexCount);
	const Vec3f& A = entityData.vertex[triangleIndices.x()];
	const Vec3f& B = entityData.vertex[triangleIndices.y()];
	const Vec3f& C = entityData.vertex[triangleIndices.z()];

	// Ray
	const int beamIdx = static_cast<int>(optixGetLaunchIndex().x);
	const int beamSampleRayIdx = static_cast<int>(optixGetPayload_0());
	const int mrSampleIdx = beamIdx * MULTI_RETURN_BEAM_SAMPLES + beamSampleRayIdx;
	const Vec3f beamSampleOrigin = optixGetWorldRayOrigin();
	const int entityId = static_cast<int>(optixGetInstanceId());
	const float laserRetro = entityData.laserRetro;

	// Hitpoint
	Vec3f hitObject = Vec3f((1 - u - v) * A + u * B + v * C);
	const Vec3f hitWorldRaytraced = optixTransformPointFromObjectToWorldSpace(hitObject);
	const Vector<3, double> hwrd = hitWorldRaytraced;
	const Vector<3, double> hso = beamSampleOrigin;
	const double distance = (hwrd - hso).length();

	// Early out for points that are too close to the sensor
	float minRange = ctx.rayRangesCount == 1 ? ctx.rayRanges[0].x() : ctx.rayRanges[optixGetLaunchIndex().x].x();
	if (distance < minRange) {
		saveSampleAsNonHit(mrSampleIdx, ctx.nearNonHitDistance);
		return;
	}

	// Normal vector
	Vec3f rayDir = optixGetWorldRayDirection();
	const Vec3f wA = optixTransformPointFromObjectToWorldSpace(A);
	const Vec3f wB = optixTransformPointFromObjectToWorldSpace(B);
	const Vec3f wC = optixTransformPointFromObjectToWorldSpace(C);
	const Vec3f wAB = wB - wA;
	const Vec3f wCA = wC - wA;
	const Vec3f wNormal = wAB.cross(wCA).normalized();

	// Incident angle
	const float cosIncidentAngle = fabs(wNormal.dot(rayDir));
	const float incidentAngle = acosf(cosIncidentAngle);

	float intensity = ctx.defaultIntensity;
	bool isIntensityRequested = ctx.intensityF32 != nullptr || ctx.intensityU8 != nullptr;
	if (isIntensityRequested && entityData.textureCoords != nullptr && entityData.texture != 0) {
		assert(triangleIndices.x() < entityData.textureCoordsCount);
		assert(triangleIndices.y() < entityData.textureCoordsCount);
		assert(triangleIndices.z() < entityData.textureCoordsCount);

		const Vec2f& uvA = entityData.textureCoords[triangleIndices.x()];
		const Vec2f& uvB = entityData.textureCoords[triangleIndices.y()];
		const Vec2f& uvC = entityData.textureCoords[triangleIndices.z()];

		Vec2f uv = (1 - u - v) * uvA + u * uvB + v * uvC;

		intensity = tex2D<TextureTexelFormat>(entityData.texture, uv[0], uv[1]);
	}
	intensity *= cosIncidentAngle;

	Vec3f absPointVelocity{NAN};
	Vec3f relPointVelocity{NAN};
	float radialSpeed{NAN};
	bool isVelocityRequested = ctx.pointAbsVelocity != nullptr || ctx.pointRelVelocity != nullptr || ctx.radialSpeed != nullptr;
	if (ctx.sceneDeltaTime > 0 && isVelocityRequested) {
		Vec3f displacementFromTransformChange = {0, 0, 0};
		if (entityData.hasPrevFrameLocalToWorld) {
			// Computing hit point velocity in simple words:
			// From raytracing, we get hit point in Entity's coordinate frame (hitObject).
			// Think of it as a marker dot on the Entity.
			// Having access to Entity's previous pose, we can compute (entityData.prevFrameLocalToWorld * hitObject),
			// where the marker dot would be in the previous raytracing frame (displacementVectorOrigin).
			// Then, we can connect marker dot in previous raytracing frame with its current position and obtain displacementFromTransformChange vector
			// Dividing displacementFromTransformChange by time elapsed from the previous raytracing frame yields velocity vector.
			Vec3f displacementVectorOrigin = entityData.prevFrameLocalToWorld * hitObject;
			displacementFromTransformChange = hitWorldRaytraced - displacementVectorOrigin;
		}

		// Some entities may have skinned meshes - in this case entity.vertexDisplacementSincePrevFrame will be non-null
		Vec3f displacementFromSkinning = {0, 0, 0};
		bool wasSkinned = entityData.vertexDisplacementSincePrevFrame != nullptr;
		if (wasSkinned) {
			Mat3x4f objectToWorld;
			optixGetObjectToWorldTransformMatrix(reinterpret_cast<float*>(objectToWorld.rc));
			// TODO(msz-rai): To verify if rotation is needed (in some tests it produces more realistic results)
			const Vec3f& vA = objectToWorld.rotation() * entityData.vertexDisplacementSincePrevFrame[triangleIndices.x()];
			const Vec3f& vB = objectToWorld.rotation() * entityData.vertexDisplacementSincePrevFrame[triangleIndices.y()];
			const Vec3f& vC = objectToWorld.rotation() * entityData.vertexDisplacementSincePrevFrame[triangleIndices.z()];
			displacementFromSkinning = objectToWorld.scaleVec() * Vec3f((1 - u - v) * vA + u * vB + v * vC);
		}

		absPointVelocity = (displacementFromTransformChange + displacementFromSkinning) /
		                   static_cast<float>(ctx.sceneDeltaTime);

		// Relative point velocity is a sum of linear velocities difference (between sensor and hit-point)
		// and impact of sensor angular velocity
		Vec3f absPointVelocityInSensorFrame = ctx.rayOriginToWorld.rotation().inverse() * absPointVelocity;
		Vec3f relPointVelocityBasedOnSensorLinearVelocity = absPointVelocityInSensorFrame - ctx.sensorLinearVelocityXYZ;

		Vec3f hitRays = ctx.rayOriginToWorld.inverse() * hitWorldRaytraced;
		Vec3f relPointVelocityBasedOnSensorAngularVelocity = Vec3f(.0f) - ctx.sensorAngularVelocityRPY.cross(hitRays);
		relPointVelocity = relPointVelocityBasedOnSensorLinearVelocity + relPointVelocityBasedOnSensorAngularVelocity;

		radialSpeed = hitRays.normalized().dot(relPointVelocity);
	}

	saveSampleAsHit(mrSampleIdx, distance, intensity, laserRetro, entityId, absPointVelocity, relPointVelocity, radialSpeed,
	                wNormal, incidentAngle);
}

extern "C" __global__ void __anyhit__() {}

// Helper functions implementations

__device__ void shootSamplingRay(const Mat3x4f& ray, float maxRange, unsigned int beamSampleRayIdx)
{
	Vec3f origin = ray * Vec3f{0, 0, 0};
	Vec3f dir = ray * Vec3f{0, 0, 1} - origin;
	const unsigned int flags = OPTIX_RAY_FLAG_DISABLE_ANYHIT; // TODO: try adding OPTIX_RAY_FLAG_CULL_BACK_FACING_TRIANGLES
	optixTrace(ctx.scene, origin, dir, 0.0f, maxRange, 0.0f, OptixVisibilityMask(255), flags, 0, 1, 0, beamSampleRayIdx);
}

__device__ Mat3x4f makeBeamSampleRayTransform(float hHalfDivergenceRad, float vHalfDivergenceRad, unsigned int layerIdx,
                                              unsigned int vertexIdx)
{
	if (ctx.hBeamHalfDivergenceRad == 0.0f && ctx.vBeamHalfDivergenceRad == 0.0f) {
		return Mat3x4f::identity();
	}

	const float hCurrentHalfDivergenceRad = hHalfDivergenceRad *
	                                        (1.0f - static_cast<float>(layerIdx) / MULTI_RETURN_BEAM_LAYERS);
	const float vCurrentHalfDivergenceRad = vHalfDivergenceRad *
	                                        (1.0f - static_cast<float>(layerIdx) / MULTI_RETURN_BEAM_LAYERS);

	const float angleStep = 2.0f * static_cast<float>(M_PI) / MULTI_RETURN_BEAM_VERTICES;

	const float hAngle = hCurrentHalfDivergenceRad * cos(static_cast<float>(vertexIdx) * angleStep);
	const float vAngle = vCurrentHalfDivergenceRad * sin(static_cast<float>(vertexIdx) * angleStep);

	return Mat3x4f::rotationRad(vAngle, 0.0f, 0.0f) * Mat3x4f::rotationRad(0.0f, hAngle, 0.0f);
}

__device__ void saveSampleAsNonHit(int sampleIdx, float nonHitDistance)
{
	ctx.mrSamples.isHit[sampleIdx] = false;
	ctx.mrSamples.distance[sampleIdx] = nonHitDistance;
}

__device__ void saveSampleAsHit(int sampleIdx, float distance, float intensity, float laserRetro, int objectID,
                                const Vec3f& absVelocity, const Vec3f& relVelocity, float radialSpeed, const Vec3f& normal,
                                float incidentAngle)
{
	ctx.mrSamples.isHit[sampleIdx] = true;
	ctx.mrSamples.distance[sampleIdx] = distance;
	ctx.mrSamples.intensity[sampleIdx] = intensity;

	if (ctx.mrSamples.laserRetro != nullptr) {
		ctx.mrSamples.laserRetro[sampleIdx] = laserRetro;
	}
	if (ctx.mrSamples.entityId != nullptr) {
		ctx.mrSamples.entityId[sampleIdx] = objectID;
	}
	if (ctx.mrSamples.absVelocity != nullptr) {
		ctx.mrSamples.absVelocity[sampleIdx] = absVelocity;
	}
	if (ctx.mrSamples.relVelocity != nullptr) {
		ctx.mrSamples.relVelocity[sampleIdx] = relVelocity;
	}
	if (ctx.mrSamples.radialSpeed != nullptr) {
		ctx.mrSamples.radialSpeed[sampleIdx] = radialSpeed;
	}
	if (ctx.mrSamples.normal != nullptr) {
		ctx.mrSamples.normal[sampleIdx] = normal;
	}
	if (ctx.mrSamples.incidentAngle != nullptr) {
		ctx.mrSamples.incidentAngle[sampleIdx] = incidentAngle;
	}
}

__device__ void saveNonHitBeamSamples(int beamIdx, float nonHitDistance)
{
	for (int sampleIdx = beamIdx * MULTI_RETURN_BEAM_SAMPLES; sampleIdx < (beamIdx + 1) * MULTI_RETURN_BEAM_SAMPLES;
	     ++sampleIdx) {
		saveSampleAsNonHit(sampleIdx, nonHitDistance);
	}
}

__device__ void saveBeamSharedData(int beamIdx, const Mat3x4f& rayLocal)
{
	for (int returnPointIdx = beamIdx * ctx.returnCount; returnPointIdx < (beamIdx + 1) * ctx.returnCount; ++returnPointIdx) {
		if (ctx.rayIdx != nullptr) {
			ctx.rayIdx[returnPointIdx] = beamIdx;
		}

		// Assuming up vector is Y, forward vector is Z (true for Unity).
		// TODO(msz-rai): allow to define up and forward vectors in RGL
		// Assuming rays are generated in left-handed coordinate system with the rotation applied in ZXY order.
		// TODO(msz-rai): move ray generation to RGL to unify rotations
		if (ctx.azimuth != nullptr) {
			ctx.azimuth[returnPointIdx] = rayLocal.toRotationYOrderZXYLeftHandRad();
		}
		if (ctx.elevation != nullptr) {
			ctx.elevation[returnPointIdx] = rayLocal.toRotationXOrderZXYLeftHandRad();
		}

		if (ctx.timestampF64 != nullptr) {
			ctx.timestampF64[returnPointIdx] = ctx.doApplyDistortion ? ctx.rayTimeOffsetsMs[beamIdx] * 1e-3f : 0; // in seconds
		}
		if (ctx.timestampU32 != nullptr) {
			ctx.timestampU32[returnPointIdx] = ctx.doApplyDistortion ?
			                                       static_cast<uint32_t>(ctx.rayTimeOffsetsMs[beamIdx] * 1e6f) // in nanoseconds
			                                       :
			                                       0;
		}
	}
}
