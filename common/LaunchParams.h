#pragma once

#include <data_types/PCLFormats.h>
#include <data_types/LidarNoiseParams.h>
#include "gdt/math/vec.h"
#include "gdt/utils/optix_macros.h"
#include "data_types/PointTypes.h"
#include "TransformMatrix.h"

#include <curand_kernel.h>

// two ray types
enum { RADIANCE_RAY_TYPE=0, RAY_TYPE_COUNT };
enum { LIDAR_RAY_TYPE=0, LIDAR_RAY_TYPE_COUNT };

struct TriangleMeshSBTData {
    // gdt::vec3f  color; // NON-POD, to be removed,
    gdt::vec3f *vertex;
    gdt::vec3f *normal;
    gdt::vec2f *texcoord;
    gdt::vec3i *index;
    size_t vertex_count;
    size_t index_count;
    size_t normal_count;
    size_t texcoord_count;
    bool                hasTexture;
    cudaTextureObject_t texture;
};

struct LaunchLidarParams
{
    float range;
    size_t rayCount;
    TransformMatrix lidarPose;
    TransformMatrix rosTransform;
    OptixTraversableHandle traversable;
    LidarNoiseParams lidarNoiseParams;

    const TransformMatrix* dRayPoses;
    Point3f* dUnityVisualisationPoints;
    PCL12* dRosXYZ;
    int* dWasHit;
    curandStatePhilox4_32_10_t* dRandomizationStates;
};
