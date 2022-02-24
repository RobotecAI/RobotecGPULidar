#pragma once

#include <data_types/PCLFormats.h>
#include "gdt/math/vec.h"
#include "gdt/utils/optix_macros.h"
#include "data_types/PointTypes.h"
#include "TransformMatrix.h"


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
    size_t rayCount;
    size_t lidarCount;
    const int* dRayCountOfLidar;
    TransformMatrix lidarPose;
    TransformMatrix postRaycastTransform;
    const TransformMatrix* dRayPoses;
    const int* dLidarArrayRingIds;
    int lidarArrayRingCount;
    const float *dRangeOfLidar;
    Point3f* dHitP3;
    PCL12* dHitXYZ;
    int* dHitIsFinite;

    OptixTraversableHandle traversable;
};
