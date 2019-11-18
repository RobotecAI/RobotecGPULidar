#pragma once

#include "gdt/math/vec.h"
#include "optix7.h"

using namespace gdt;

// two ray types
enum { RADIANCE_RAY_TYPE=0, SHADOW_RAY_TYPE, RAY_TYPE_COUNT };
enum { LIDAR_RAY_TYPE=0, LIDAR_RAY_TYPE_COUNT };

struct TriangleMeshSBTData {
    vec3f  color;
    vec3f *vertex;
    vec3f *normal;
    vec2f *texcoord;
    vec3i *index;
    bool                hasTexture;
    cudaTextureObject_t texture;
};

// everything we want to move between host and device must be in this structure
struct LaunchParams
{
    struct {
        uint32_t *colorBuffer;
        float    *lidarBuffer; // buffer for lidar data on device
        vec2i     size;
        uint32_t  lidarSize;
    } frame;

    struct {
        vec3f position;
        vec3f direction;
        vec3f horizontal;
        vec3f vertical;
    } camera;

    OptixTraversableHandle traversable;
};


struct LaunchLidarParams
{
    float *rayBuffer;
    float *positionBuffer;
    int    fbSize;
    int   *hitBuffer;
    float range;

    OptixTraversableHandle traversable;
};
