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
template<bool isFinite>
__device__ void saveRayResult(const Vec3f& xyz, float distance, float intensity, int objectID, const Vec3f& absVelocity,
                              const Vec3f& relVelocity, float radialSpeed, const Vec3f& normal, float incidentAngle);
__device__ void saveNonHitRayResult(float nonHitDistance);
__device__ void shootSamplingRay(const Mat3x4f& ray, float maxRange, unsigned sampleBeamIdx);
__device__ Mat3x4f makeBeamSampleRayTransform(float hHalfDivergenceAngleRad, float vHalfVDivergenceAngleRad, unsigned ellipseIdx, unsigned vertexIdx);

extern "C" __global__ void __raygen__()
{
	if (ctx.scene == 0) {
		saveNonHitRayResult(
		    ctx.farNonHitDistance); // TODO(prybicki): Remove this, assume that host code will not pass invalid scene.
		return;
	}

	const int rayIdx = static_cast<int>(optixGetLaunchIndex().x);

	if (ctx.rayMask != nullptr && ctx.rayMask[rayIdx] == 0) {
		saveNonHitRayResult(ctx.farNonHitDistance);
		return;
	}

	Mat3x4f ray = ctx.raysWorld[rayIdx];
	const Mat3x4f rayLocal =
	    ctx.rayOriginToWorld.inverse() *
	    ray; // TODO(prybicki): instead of computing inverse, we should pass rays in local CF and then transform them to world CF.

	// Assuming up vector is Y, forward vector is Z (true for Unity).
	// TODO(msz-rai): allow to define up and forward vectors in RGL
	// Assuming rays are generated in left-handed coordinate system with the rotation applied in ZXY order.
	// TODO(msz-rai): move ray generation to RGL to unify rotations
	if (ctx.azimuth != nullptr) {
		ctx.azimuth[rayIdx] = rayLocal.toRotationYOrderZXYLeftHandRad();
	}
	if (ctx.elevation != nullptr) {
		ctx.elevation[rayIdx] = rayLocal.toRotationXOrderZXYLeftHandRad();
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
	if (ctx.hBeamHalfDivergence > 0.0f && ctx.vBeamHalfDivergence > 0.0f) {
		// Shoot multi-return sampling rays
		for (int ellipseIdx = 0; ellipseIdx < MULTI_RETURN_BEAM_ELLIPSES; ++ellipseIdx) {
			for (int vertexIdx = 0; vertexIdx < MULTI_RETURN_BEAM_VERTICES; vertexIdx++) {
				Mat3x4f sampleRay = ray * makeBeamSampleRayTransform(ctx.hBeamHalfDivergence, ctx.vBeamHalfDivergence, ellipseIdx, vertexIdx);
				// Sampling rays indexes start from 1, 0 is reserved for the primary ray
				const unsigned beamSampleRayIdx = 1 + ellipseIdx * MULTI_RETURN_BEAM_VERTICES + vertexIdx;
				shootSamplingRay(sampleRay, maxRange, beamSampleRayIdx);
			}
		}
	}
}

extern "C" __global__ void __miss__()
{
	const int beamIdx = static_cast<int>(optixGetLaunchIndex().x);
	const int beamSampleIdx = static_cast<int>(optixGetPayload_0());
	ctx.mrSamples.isHit[beamIdx * MULTI_RETURN_BEAM_SAMPLES + beamSampleIdx] = false;
	if (beamSampleIdx == 0) {
		saveNonHitRayResult(ctx.farNonHitDistance);
	}
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
	const unsigned beamSampleRayIdx = optixGetPayload_0();
	const unsigned circleIdx = (beamSampleRayIdx - 1) / MULTI_RETURN_BEAM_VERTICES;
	const unsigned vertexIdx = (beamSampleRayIdx - 1) % MULTI_RETURN_BEAM_VERTICES;
	const unsigned mrSamplesIdx = beamIdx * MULTI_RETURN_BEAM_SAMPLES + beamSampleRayIdx;
	const Vec3f beamSampleOrigin = optixGetWorldRayOrigin();
	const int entityId = static_cast<int>(optixGetInstanceId());

	// Hitpoint
	Vec3f hitObject = Vec3f((1 - u - v) * A + u * B + v * C);
	const Vec3f hitWorldRaytraced = optixTransformPointFromObjectToWorldSpace(hitObject);
	const Vector<3, double> hwrd = hitWorldRaytraced;
	const Vector<3, double> hso = beamSampleOrigin;
	const double distance = (hwrd - hso).length();
	const Vec3f hitWorldSeenBySensor = [&]() {
		if (!ctx.doApplyDistortion) {
			return hitWorldRaytraced;
		}
		Mat3x4f sampleRayTf = beamSampleRayIdx == 0 ? Mat3x4f::identity() :
		                                              makeBeamSampleRayTransform(ctx.hBeamHalfDivergence, ctx.vBeamHalfDivergence, circleIdx, vertexIdx);
		Mat3x4f undistortedRay = ctx.raysWorld[beamIdx] * sampleRayTf;
		Vec3f undistortedOrigin = undistortedRay * Vec3f{0, 0, 0};
		Vec3f undistortedDir = undistortedRay * Vec3f{0, 0, 1} - undistortedOrigin;
		return undistortedOrigin + undistortedDir * distance;
	}();

	// Early out for points that are too close to the sensor
	float minRange = ctx.rayRangesCount == 1 ? ctx.rayRanges[0].x() : ctx.rayRanges[optixGetLaunchIndex().x].x();
	if (distance < minRange) {
		ctx.mrSamples.isHit[mrSamplesIdx] = false;
		if (beamSampleRayIdx == 0) {
			saveNonHitRayResult(ctx.nearNonHitDistance);
		}
		return;
	}

	// Normal vector and incident angle
	Vec3f rayDir = optixGetWorldRayDirection();
	const Vec3f wA = optixTransformPointFromObjectToWorldSpace(A);
	const Vec3f wB = optixTransformPointFromObjectToWorldSpace(B);
	const Vec3f wC = optixTransformPointFromObjectToWorldSpace(C);
	const Vec3f wAB = wB - wA;
	const Vec3f wCA = wC - wA;
	const Vec3f wNormal = wAB.cross(wCA).normalized();
	const float incidentAngle = acosf(fabs(wNormal.dot(rayDir)));

	float intensity = 0;
	bool isIntensityRequested = ctx.intensity != nullptr;
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

	// Save sub-sampling results
	ctx.mrSamples.isHit[mrSamplesIdx] = true;
	ctx.mrSamples.distance[mrSamplesIdx] = distance;
	if (beamSampleRayIdx != 0) {
		return;
	}

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
			const Vec3f& vA = entityData.vertexDisplacementSincePrevFrame[triangleIndices.x()];
			const Vec3f& vB = entityData.vertexDisplacementSincePrevFrame[triangleIndices.y()];
			const Vec3f& vC = entityData.vertexDisplacementSincePrevFrame[triangleIndices.z()];
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
	saveRayResult<true>(hitWorldSeenBySensor, distance, intensity, entityId, absPointVelocity, relPointVelocity, radialSpeed,
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

__device__ Mat3x4f makeBeamSampleRayTransform(float hHalfDivergenceAngleRad, float vHalfDivergenceAngleRad, unsigned int ellipseIdx, unsigned int vertexIdx)
{
	if (ctx.hBeamHalfDivergence == 0.0f && ctx.vBeamHalfDivergence == 0.0f) {
		return Mat3x4f::identity();
	}
//	const auto circleDivergence = hHalfDivergenceAngleRad * (1.0f - static_cast<float>(ellipseIdx) / MULTI_RETURN_BEAM_ELLIPSES);
//	auto vertexAngleStep = 2.0f * static_cast<float>(M_PI) / MULTI_RETURN_BEAM_VERTICES;
//	// First, rotate around X to move the ray vector (initially {0,0,1}) to diverge from the Z axis by circleDivergence.
//	// Then rotate around Z to move the ray vector on the circle.
//	return Mat3x4f::rotationRad(0, 0, static_cast<float>(vertexIdx) * vertexAngleStep) * // Second
//	       Mat3x4f::rotationRad(circleDivergence, 0, 0);

	const float hCurrentDivergence = hHalfDivergenceAngleRad * (1.0f - static_cast<float>(ellipseIdx) / MULTI_RETURN_BEAM_ELLIPSES);
	const float vCurrentDivergence = vHalfDivergenceAngleRad * (1.0f - static_cast<float>(ellipseIdx) / MULTI_RETURN_BEAM_ELLIPSES);

	const float angleStep = 2.0f * static_cast<float>(M_PI) / MULTI_RETURN_BEAM_VERTICES;

	const float hAngle = hCurrentDivergence * cos(static_cast<float>(vertexIdx) * angleStep);
	const float vAngle = vCurrentDivergence * sin(static_cast<float>(vertexIdx) * angleStep);

	return Mat3x4f::rotationRad(vAngle, 0.0f, 0.0f) * Mat3x4f::rotationRad(0.0f, hAngle, 0.0f);
}

__device__ void saveNonHitRayResult(float nonHitDistance)
{
	Mat3x4f ray = ctx.raysWorld[optixGetLaunchIndex().x];
	Vec3f origin = ray * Vec3f{0, 0, 0};
	Vec3f dir = ray * Vec3f{0, 0, 1} - origin;
	Vec3f displacement = dir.normalized() * nonHitDistance;
	displacement = {isnan(displacement.x()) ? 0 : displacement.x(), isnan(displacement.y()) ? 0 : displacement.y(),
	                isnan(displacement.z()) ? 0 : displacement.z()};
	Vec3f xyz = origin + displacement;
	saveRayResult<false>(xyz, nonHitDistance, 0, RGL_ENTITY_INVALID_ID, Vec3f{NAN}, Vec3f{NAN}, 0.001f, Vec3f{NAN}, NAN);
}

template<bool isFinite>
__device__ void saveRayResult(const Vec3f& xyz, float distance, float intensity, const int objectID, const Vec3f& absVelocity,
                              const Vec3f& relVelocity, float radialSpeed, const Vec3f& normal, float incidentAngle)
{
	const int beamIdx = static_cast<int>(optixGetLaunchIndex().x);
	if (ctx.xyz != nullptr) {
		// Return actual XYZ of the hit point or infinity vector.
		ctx.xyz[beamIdx] = xyz;
	}
	if (ctx.isHit != nullptr) {
		ctx.isHit[beamIdx] = isFinite;
	}
	if (ctx.rayIdx != nullptr) {
		ctx.rayIdx[beamIdx] = beamIdx;
	}
	if (ctx.ringIdx != nullptr && ctx.ringIds != nullptr) {
		ctx.ringIdx[beamIdx] = ctx.ringIds[beamIdx % ctx.ringIdsCount];
	}
	if (ctx.distance != nullptr) {
		ctx.distance[beamIdx] = distance;
	}
	if (ctx.intensity != nullptr) {
		ctx.intensity[beamIdx] = intensity;
	}
	if (ctx.timestamp != nullptr) {
		ctx.timestamp[beamIdx] = ctx.sceneTime;
	}
	if (ctx.entityId != nullptr) {
		ctx.entityId[beamIdx] = isFinite ? objectID : RGL_ENTITY_INVALID_ID;
	}
	if (ctx.pointAbsVelocity != nullptr) {
		ctx.pointAbsVelocity[beamIdx] = absVelocity;
	}
	if (ctx.pointRelVelocity != nullptr) {
		ctx.pointRelVelocity[beamIdx] = relVelocity;
	}
	if (ctx.radialSpeed != nullptr) {
		ctx.radialSpeed[beamIdx] = radialSpeed;
	}
	if (ctx.normal != nullptr) {
		ctx.normal[beamIdx] = normal;
	}
	if (ctx.incidentAngle != nullptr) {
		ctx.incidentAngle[beamIdx] = incidentAngle;
	}
}
