#pragma once

#include <optix.h>

#include <data_types/PCLFormats.h>
#include <data_types/LidarNoiseParams.h>
#include <data_types/PointTypes.h>
#include <math/Mat3x4f.hpp>

#include <curand_kernel.h>

// two ray types
enum { RADIANCE_RAY_TYPE=0, RAY_TYPE_COUNT };
enum { LIDAR_RAY_TYPE=0, LIDAR_RAY_TYPE_COUNT };

struct LaunchLidarParams
{
    float range;
    size_t rayCount;
    Mat3x4f lidarPose;
    Mat3x4f rosTransform;
    OptixTraversableHandle traversable;
    LidarNoiseParams lidarNoiseParams;

    const Mat3x4f* dRayPoses;
    Point3f* dUnityVisualisationPoints;
    PCL12* dRosXYZ;
    int* dWasHit;
    curandStatePhilox4_32_10_t* dRandomizationStates;
};
