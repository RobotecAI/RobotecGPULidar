#pragma once

#include "formatPCL.h"
#include "gaussianNoise.h"
#include <random>

#include <Logger.h>
#include <APIObject.hpp>
#include <scene/Scene.hpp>
#include <Optix.hpp>

#include <rgl/api/experimental.h>
#include <rgl/api/e2e_extensions.h>

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
    Lidar(TransformMatrix* rayPoses, int rayPosesCount);

    void setRingIds(int* ringIds, size_t ringIdsCount);
    void scheduleRaycast(std::shared_ptr<Scene> scene);
    int getResultsSize();
    void getResults(int format, void* data);

    LidarNoiseParams lidarNoiseParams {};
    TransformMatrix lidarPose = TransformMatrix::identity();
    TransformMatrix rosTransform{};
    float range;

private:
    cudaStream_t stream = nullptr;
    std::optional<LaunchLidarParams> currentJob;
    std::optional<int> densePointCount;

    // GPU INPUT
    DeviceBuffer<LaunchLidarParams> NAMED(dCurrentJob);
    DeviceBuffer<TransformMatrix> NAMED(dRayPoses);
    DeviceBuffer<int> NAMED(dLidarArrayRingIds);

    // GPU OUTPUT
    DeviceBuffer<int> NAMED(dWasHit);
    DeviceBuffer<int> NAMED(dHitsBeforeIndex);

    DeviceBuffer<Point3f> NAMED(dUnityVisualisationPoints); // Native output (for visualization in Unity)
    DeviceBuffer<PCL12> NAMED(dRosXYZ); // Native output (for publishing via ROS), contains non-hits

    // Buffers to be computed via formatPCLs
    DeviceBuffer<Point3f> NAMED(dDensePoint3f);
    DeviceBuffer<PCL12> NAMED(dDensePCL12);
    DeviceBuffer<PCL24> NAMED(dDensePCL24);
    DeviceBuffer<PCL48> NAMED(dDensePCL48);

    HostPinnedBuffer<Point3f> NAMED(hDensePoint3f);
    HostPinnedBuffer<PCL12> NAMED(hDensePCL12);
    HostPinnedBuffer<PCL24> NAMED(hDensePCL24);
    HostPinnedBuffer<PCL48> NAMED(hDensePCL48);

    DeviceBuffer<curandStatePhilox4_32_10_t> NAMED(dRandomizationStates);
    std::random_device randomDevice;
};
