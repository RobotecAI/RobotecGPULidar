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
__forceinline__ __device__ void saveRayResult(const Vec3f* xyz = nullptr, float distance = NON_HIT_VALUE, float intensity = 0,
                                              const int objectID = RGL_ENTITY_INVALID_ID, const Vec3f velocity = Vec3f{NAN})
{
	const int rayIdx = optixGetLaunchIndex().x;
	if (ctx.xyz != nullptr) {
		// Return actual XYZ of the hit point or infinity vector.
		ctx.xyz[rayIdx] = isFinite ? *xyz : Vec3f{NON_HIT_VALUE, NON_HIT_VALUE, NON_HIT_VALUE};
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
		ctx.pointAbsVelocity[rayIdx] = velocity;
	}
}

extern "C" __global__ void __raygen__()
{
	if (ctx.scene == 0) {
		saveRayResult<false>();
		return;
	}

	const int rayIdx = optixGetLaunchIndex().x;
	Mat3x4f ray = ctx.rays[rayIdx];

	if (ctx.doApplyDistortion) {
		static const float toDeg = (180.0f / M_PI);
		// Velocities are in the local frame. Need to transform rays.
		ray = ctx.rayOriginToWorld.inverse() * ray;
		// Ray time offsets are in milliseconds, velocities are in unit per seconds.
		// In order to not lose numerical precision, first multiply values and then convert to proper unit.
		ray = Mat3x4f::TRS((ctx.rayTimeOffsets[rayIdx] * ctx.sensorLinearVelocityXYZ) * 0.001f,
		                   (ctx.rayTimeOffsets[rayIdx] * (ctx.sensorAngularVelocityRPY * toDeg)) * 0.001f) *
		      ray;
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
	const Vec3i index = entityData.index[primID];
	const float u = optixGetTriangleBarycentrics().x;
	const float v = optixGetTriangleBarycentrics().y;

	assert(index.x() < entityData.vertexCount);
	assert(index.y() < entityData.vertexCount);
	assert(index.z() < entityData.vertexCount);

	const Vec3f& A = entityData.vertex[index.x()];
	const Vec3f& B = entityData.vertex[index.y()];
	const Vec3f& C = entityData.vertex[index.z()];

	Vec3f hitObject = Vec3f((1 - u - v) * A + u * B + v * C);
	Vec3f hitWorld = optixTransformPointFromObjectToWorldSpace(hitObject);

	int objectID = optixGetInstanceId();

	Vec3f origin = decodePayloadVec3f({optixGetPayload_0(), optixGetPayload_1(), optixGetPayload_2()});

	// TODO: Optimization - we can use inversesqrt here, which is one operation cheaper then sqrt.
	float distance = sqrt(pow((hitWorld)[0] - (origin)[0], 2) + pow((hitWorld)[1] - (origin)[1], 2) +
	                      pow((hitWorld)[2] - (origin)[2], 2));

	float minRange = ctx.rayRangesCount == 1 ? ctx.rayRanges[0].x() : ctx.rayRanges[optixGetLaunchIndex().x].x();
	if (distance < minRange) {
		saveRayResult<false>();
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

	float intensity = 0;
	if (entityData.textureCoords != nullptr && entityData.texture != 0) {
		assert(index.x() < entityData.textureCoordsCount);
		assert(index.y() < entityData.textureCoordsCount);
		assert(index.z() < entityData.textureCoordsCount);

		const Vec2f& uvA = entityData.textureCoords[index.x()];
		const Vec2f& uvB = entityData.textureCoords[index.y()];
		const Vec2f& uvC = entityData.textureCoords[index.z()];

		Vec2f uv = (1 - u - v) * uvA + u * uvB + v * uvC;

		intensity = tex2D<TextureTexelFormat>(entityData.texture, uv[0], uv[1]);
	}

	Vec3f velocity{NAN};
	if (ctx.sceneDeltaTime > 0 && entityData.hasPrevFrameLocalToWorld) {
		// Computing hit point velocity in simple words:
		// From raytracing, we get hit point in Entity's coordinate frame (hitObject).
		// Think of it as a marker dot on the Entity.
		// Having access to Entity's previous pose, we can compute (entityData.prevFrameLocalToWorld * hitObject),
		// where the marker dot would be in the previous raytracing frame (displacementOriginWorld).
		// Then, we can connect marker dot in previous raytracing frame with its current position and obtain displacement vector
		// Dividing displacement by time elapsed from the previous raytracing frame yields velocity vector.
		Vec3f displacementOriginWorld = entityData.prevFrameLocalToWorld * hitObject;
		Vec3f displacement = hitWorld - displacementOriginWorld;
		velocity = displacement / static_cast<float>(ctx.sceneDeltaTime);
	}

	saveRayResult<true>(&hitWorld, distance, intensity, objectID, velocity);
}

extern "C" __global__ void __miss__() { saveRayResult<false>(); }

extern "C" __global__ void __anyhit__() {}
