#pragma once

#include <math/Vector.hpp>
#include <optix.h>

struct TriangleMeshSBTData {
	const Vec3f *vertex;
	const Vec3i *index;
	const Vec2f *tex_coord;
	size_t vertex_count;
	size_t index_count;
	size_t tex_coord_count;

	cudaTextureObject_t texture;

	size_t texture_width;
	size_t texture_height;
};


struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) RaygenRecord {
    char header[OPTIX_SBT_RECORD_HEADER_SIZE];
};

struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) MissRecord {
    char header[OPTIX_SBT_RECORD_HEADER_SIZE];
};

struct __align__(OPTIX_SBT_RECORD_ALIGNMENT) HitgroupRecord {
    char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    TriangleMeshSBTData data;
};
