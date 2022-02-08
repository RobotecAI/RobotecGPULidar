#pragma once

#include "gdt/math/vec.h"
#include "gdt/utils/optix_macros.h"
#include "PointTypes.h"

using namespace gdt;

// two ray types
enum { RADIANCE_RAY_TYPE=0, RAY_TYPE_COUNT };
enum { LIDAR_RAY_TYPE=0, LIDAR_RAY_TYPE_COUNT };

struct TriangleMeshSBTData {
    // vec3f  color; // NON-POD, to be removed,
    vec3f *vertex;
    vec3f *normal;
    vec2f *texcoord;
    vec3i *index;
    size_t vertex_count;
    size_t index_count;
    size_t normal_count;
    size_t texcoord_count;
    bool                hasTexture;
    cudaTextureObject_t texture;
};


struct LaunchLidarParams
{
    size_t rayCount;
    size_t lidarCount;
    const int* rayCountOfLidar;
    const Point3f* rayDirs;
    const float *rangeOfLidar;
    const Point3f* positionOfLidar;
    
    LidarPoint* hitXYZI;
    int* hitIsFinite;

    OptixTraversableHandle traversable;
};
