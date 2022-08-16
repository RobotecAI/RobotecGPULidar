#pragma once

#include "formatPCL.h"
#include "gaussianNoise.h"
#include <random>

#include <Logger.h>
#include <APIObject.hpp>
#include <scene/Scene.hpp>
#include <gpu/Optix.hpp>

#include <rgl/api/experimental.h>

// Currently there is no explicit (documented) model of API / GPU synchronization
// Getting correct results is dependent on calling API calls in right order
// Failure to do so will likely result in segfaults / incorrect results
// TODO: fix this

/**
 * Lidar represents the following:
 * - Description of a lidar model (list of ray transforms); a rough equivalent of the lidar's tech spec
 * - Potential ray-casting and post-processing happening asynchronously (encapsulates a cudaStream)
 * - Memory allocations required for the above
 *
 *  In the future, the description part might be separated
 *  to avoid data duplication when multiple lidars of the same type are used.
 */
struct Lidar : APIObject<Lidar>
{
    Lidar(const Mat3x4f* rayPoses, int rayPosesCount);

    void setRingIds(const int* ringIds, size_t ringIdsCount);
    void scheduleRaycast(std::shared_ptr<Scene> scene);
    int getResultsSize();
    void getResults(int format, void* data);

    Mat3x4f lidarPose = Mat3x4f::identity();
    Mat3x4f rosTransform{};
    float range;

private:
    cudaStream_t stream = nullptr;
    std::optional<LaunchLidarParams> currentJob;
    std::optional<int> densePointCount;

    // GPU INPUT
    DeviceBuffer<LaunchLidarParams> dCurrentJob;
    DeviceBuffer<Mat3x4f> dRayPoses;
    DeviceBuffer<int> dLidarArrayRingIds;

    // GPU OUTPUT
    DeviceBuffer<int> dWasHit;
    DeviceBuffer<int> dHitsBeforeIndex;

    DeviceBuffer<Point3f> dUnityVisualisationPoints; // Native output (for visualization in Unity)
    DeviceBuffer<PCL12> dRosXYZ; // Native output (for publishing via ROS), contains non-hits

    // Buffers to be computed via formatPCLs
    DeviceBuffer<Point3f> dDensePoint3f;
    DeviceBuffer<PCL12> dDensePCL12;
    DeviceBuffer<PCL24> dDensePCL24;
    DeviceBuffer<PCL48> dDensePCL48;

    HostPinnedBuffer<Point3f> hDensePoint3f;
    HostPinnedBuffer<PCL12> hDensePCL12;
    HostPinnedBuffer<PCL24> hDensePCL24;
    HostPinnedBuffer<PCL48> hDensePCL48;

    DeviceBuffer<curandStatePhilox4_32_10_t> dRandomizationStates;
    std::random_device randomDevice;
};
