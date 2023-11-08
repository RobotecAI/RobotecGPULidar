#pragma once

#include <math/Vector.hpp>
#include <math/Mat3x4f.hpp>
#include <optix.h>

/**
 * This struct is a part of HitGroupRecord in SBT (Shader Binding Table).
 * This data is available in raytracing callbacks.
 */
struct EntitySBTData
{
	const Vec3f* vertex;
	const Vec3i* index;
	size_t vertexCount;
	size_t indexCount;

	const Vec2f* textureCoords;
	size_t textureCoordsCount;
	cudaTextureObject_t texture;

	// Info about the previous frame:
	float prevFrameTimeDiff;       // Zero, if previous pose is not available or stale, or if scene time has not been updated.
	Mat3x4f prevFrameLocalToWorld; // If prevFrameTimeDiff == 0, prevFrameLocalToWorld should not be used.
};
static_assert(std::is_trivially_copyable<EntitySBTData>::value);
static_assert(std::is_trivially_constructible<EntitySBTData>::value);


struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) RaygenRecord { char header[OPTIX_SBT_RECORD_HEADER_SIZE]; };

struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) MissRecord { char header[OPTIX_SBT_RECORD_HEADER_SIZE]; };

struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) HitgroupRecord
{
	char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	EntitySBTData data;
};
