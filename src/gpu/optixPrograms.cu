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

static constexpr float toDeg = (180.0f / M_PI);

extern "C" static __constant__ RaytraceRequestContext ctx;

struct Vec3fPayload
{
	unsigned p0;
	unsigned p1;
	unsigned p2;
};

__forceinline__ __device__ Vec3fPayload encodePayloadVec3f(const Vec3f& src)
{
	Vec3fPayload dst;
	dst.p0 = *(reinterpret_cast<const unsigned*>(&src[0]));
	dst.p1 = *(reinterpret_cast<const unsigned*>(&src[1]));
	dst.p2 = *(reinterpret_cast<const unsigned*>(&src[2]));
	return dst;
}

__forceinline__ __device__ Vec3f decodePayloadVec3f(const Vec3fPayload& src)
{
	return Vec3f{
	    *reinterpret_cast<const float*>(&src.p0),
	    *reinterpret_cast<const float*>(&src.p1),
	    *reinterpret_cast<const float*>(&src.p2),
	};
}

template<bool isFinite>
__forceinline__ __device__ void saveRayResult(const Vec3f& xyz, float distance, float intensity, const int objectID,
                                              const Vec3f& absVelocity, const Vec3f& relVelocity, float radialSpeed,
                                              const Vec3f& normal, float incidentAngle)
{
	const int rayIdx = optixGetLaunchIndex().x;
	if (ctx.xyz != nullptr) {
		// Return actual XYZ of the hit point or infinity vector.
		ctx.xyz[rayIdx] = xyz;
	}
	if (ctx.isHit != nullptr) {
		ctx.isHit[rayIdx] = isFinite;
	}
	if (ctx.rayIdx != nullptr) {
		ctx.rayIdx[rayIdx] = rayIdx;
	}
	if (ctx.ringIdx != nullptr && ctx.ringIds != nullptr) {
		ctx.ringIdx[rayIdx] = ctx.ringIds[rayIdx % ctx.ringIdsCount];
	}
	if (ctx.distance != nullptr) {
		ctx.distance[rayIdx] = distance;
	}
	if (ctx.intensity != nullptr) {
		ctx.intensity[rayIdx] = intensity;
	}
	if (ctx.timestamp != nullptr) {
		ctx.timestamp[rayIdx] = ctx.sceneTime;
	}
	if (ctx.entityId != nullptr) {
		ctx.entityId[rayIdx] = isFinite ? objectID : RGL_ENTITY_INVALID_ID;
	}
	if (ctx.pointAbsVelocity != nullptr) {
		ctx.pointAbsVelocity[rayIdx] = absVelocity;
	}
	if (ctx.pointRelVelocity != nullptr) {
		ctx.pointRelVelocity[rayIdx] = relVelocity;
	}
	if (ctx.radialSpeed != nullptr) {
		ctx.radialSpeed[rayIdx] = radialSpeed;
	}
	if (ctx.normal != nullptr) {
		ctx.normal[rayIdx] = normal;
	}
	if (ctx.incidentAngle != nullptr) {
		ctx.incidentAngle[rayIdx] = incidentAngle;
	}
}

__forceinline__ __device__ void saveNonHitRayResult(float nonHitDistance)
{
	Mat3x4f ray = ctx.rays[optixGetLaunchIndex().x];
	Vec3f origin = ray * Vec3f{0, 0, 0};
	Vec3f dir = ray * Vec3f{0, 0, 1} - origin;
	Vec3f xyz = origin + dir.normalized() * nonHitDistance;
	xyz = {isnan(xyz.x()) ? 0 : xyz.x(), isnan(xyz.y()) ? 0 : xyz.y(), isnan(xyz.z()) ? 0 : xyz.z()};
	saveRayResult<false>(xyz, nonHitDistance, 0, RGL_ENTITY_INVALID_ID, Vec3f{NAN}, Vec3f{NAN}, NAN, Vec3f{NAN}, NAN);
}

extern "C" __global__ void __raygen__()
{
	if (ctx.scene == 0) {
		saveNonHitRayResult(ctx.farNonHitDistance);
		return;
	}

	const int rayIdx = optixGetLaunchIndex().x;
	Mat3x4f ray = ctx.rays[rayIdx];
	const Mat3x4f rayLocal = ctx.rayOriginToWorld.inverse() * ray;

	// Assuming up vector is Y, forward vector is Z (true for Unity).
	// TODO(msz-rai): allow to define up and forward vectors in RGL
	if (ctx.azimuth != nullptr) {
		ctx.azimuth[rayIdx] = rayLocal.toRotationYRad();
	}
	if (ctx.elevation != nullptr) {
		ctx.elevation[rayIdx] = rayLocal.toRotationXRad();
	}

	if (ctx.doApplyDistortion) {
		// Velocities are in the local frame. Need to operate on rays in local frame.
		// Ray time offsets are in milliseconds, velocities are in unit per seconds.
		// In order to not lose numerical precision, first multiply values and then convert to proper unit.
		ray = Mat3x4f::TRS((ctx.rayTimeOffsets[rayIdx] * ctx.sensorLinearVelocityXYZ) * 0.001f,
		                   (ctx.rayTimeOffsets[rayIdx] * (ctx.sensorAngularVelocityRPY * toDeg)) * 0.001f) *
		      rayLocal;
		// Back to the global frame.
		ray = ctx.rayOriginToWorld * ray;
	}

	Vec3f origin = ray * Vec3f{0, 0, 0};
	Vec3f dir = ray * Vec3f{0, 0, 1} - origin;
	float maxRange = ctx.rayRangesCount == 1 ? ctx.rayRanges[0].y() : ctx.rayRanges[rayIdx].y();

	unsigned int flags = OPTIX_RAY_FLAG_DISABLE_ANYHIT;
	Vec3fPayload originPayload = encodePayloadVec3f(origin);
	optixTrace(ctx.scene, origin, dir, 0.0f, maxRange, 0.0f, OptixVisibilityMask(255), flags, 0, 1, 0, originPayload.p0,
	           originPayload.p1, originPayload.p2);
}

extern "C" __global__ void __closesthit__()
{
	const EntitySBTData& entityData = *(const EntitySBTData*) optixGetSbtDataPointer();

	const int primID = optixGetPrimitiveIndex();
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

	Vec3f hitObject = Vec3f((1 - u - v) * A + u * B + v * C);
	Vec3f hitWorld = optixTransformPointFromObjectToWorldSpace(hitObject);

	int objectID = optixGetInstanceId();

	Vec3f origin = decodePayloadVec3f({optixGetPayload_0(), optixGetPayload_1(), optixGetPayload_2()});

	// TODO: Optimization - we can use inversesqrt here, which is one operation cheaper then sqrt.
	float distance = sqrt(pow((hitWorld)[0] - (origin)[0], 2) + pow((hitWorld)[1] - (origin)[1], 2) +
	                      pow((hitWorld)[2] - (origin)[2], 2));

	float minRange = ctx.rayRangesCount == 1 ? ctx.rayRanges[0].x() : ctx.rayRanges[optixGetLaunchIndex().x].x();
	if (distance < minRange) {
		saveNonHitRayResult(ctx.nearNonHitDistance);
		return;
	}

	// Fix XYZ if distortion is applied (XYZ must be calculated in sensor coordinate frame)
	if (ctx.doApplyDistortion) {
		const int rayIdx = optixGetLaunchIndex().x;
		Mat3x4f undistortedRay = ctx.rays[rayIdx];
		Vec3f undistortedOrigin = undistortedRay * Vec3f{0, 0, 0};
		Vec3f undistortedDir = undistortedRay * Vec3f{0, 0, 1} - undistortedOrigin;
		hitWorld = undistortedOrigin + undistortedDir * distance;
	}

	// Normal vector and incident angle
	Vec3f rayDir = (hitWorld - origin).normalized();
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
			displacementFromTransformChange = hitWorld - displacementVectorOrigin;
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

		Vec3f distanceOnAxisXYZ = hitWorld - origin;
		Vec3f relPointVelocityBasedOnSensorAngularVelocity = Vec3f(.0f) - ctx.sensorAngularVelocityRPY.cross(distanceOnAxisXYZ);
		relPointVelocity = relPointVelocityBasedOnSensorLinearVelocity + relPointVelocityBasedOnSensorAngularVelocity;

		Vec3f hitRays = ctx.rayOriginToWorld.inverse() * hitWorld;
		radialSpeed = hitRays.normalized().dot(relPointVelocity);
	}

	saveRayResult<true>(hitWorld, distance, intensity, objectID, absPointVelocity, relPointVelocity, radialSpeed, wNormal,
	                    incidentAngle);
}

extern "C" __global__ void __miss__() { saveNonHitRayResult(ctx.farNonHitDistance); }

extern "C" __global__ void __anyhit__() {}
