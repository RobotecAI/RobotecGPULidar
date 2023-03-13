#pragma once

#include <math/Vector.hpp>
#include <optix.h>

struct TriangleMeshSBTData
{
	const Vec3f* vertex;
	const Vec3i* index;
	size_t vertex_count;
	size_t index_count;
};


struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) RaygenRecord { char header[OPTIX_SBT_RECORD_HEADER_SIZE]; };

struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) MissRecord { char header[OPTIX_SBT_RECORD_HEADER_SIZE]; };

struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) HitgroupRecord
{
	char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	TriangleMeshSBTData data;
};
