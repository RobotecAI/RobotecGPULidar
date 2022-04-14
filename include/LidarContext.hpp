#pragma once

#include "formatPCL.h"
#include "gaussianNoise.h"
#include <random>

/**
 * LidarContext represents the following:
 * - Description of a lidar model (list of ray transforms); a rough equivalent of the lidar's tech spec
 * - Potential ray-casting and post-processing happening asynchronously (encapsulates a cudaStream)
 * - Memory allocations required for the above
 *
 *  In the future, the description part might be separated
 *  to avoid data duplication when multiple lidars of the same type are used.
 */
struct LidarContext
{
    cudaStream_t stream = nullptr;

    LidarContext(TransformMatrix* rayPoses, int rayPosesCount, int* lidarArrayRingIds, int lidarArrayRingCount)
    {
        CUDA_CHECK(StreamCreate(&stream));

        if (rayPosesCount <= 0) {
            auto msg = fmt::format("LidarContext::LidarContext: rayPosesCount ({}) must be > 0", rayPosesCount);
            throw std::logic_error(msg);
        }

        if (lidarArrayRingCount <= 0) {
            auto msg = fmt::format("LidarContext::LidarContext: lidarArrayRingCount ({}) must be > 0", lidarArrayRingCount);
            throw std::logic_error(msg);
        }

        dRayPoses.copyFromHost(rayPoses, rayPosesCount);
        dLidarArrayRingIds.copyFromHost(lidarArrayRingIds, lidarArrayRingCount);

        dUnityVisualisationPoints.resizeToFit(rayPosesCount);
        dRosXYZ.resizeToFit(rayPosesCount);
        dDensePoint3f.resizeToFit(rayPosesCount);
        dDensePCL12.resizeToFit(rayPosesCount);
        dDensePCL24.resizeToFit(rayPosesCount);
        dDensePCL48.resizeToFit(rayPosesCount);
        dWasHit.resizeToFit(rayPosesCount);
        dHitsBeforeIndex.resizeToFit(rayPosesCount);

        dRandomizationStates.resizeToFit(rayPosesCount);
        setupGaussianNoiseGenerator(randomDevice(), stream, dRandomizationStates);
    }

    // TODO: Arguments such as OptiX Pipeline and SBT should be passed coupled (as structure)
    void scheduleRaycast(OptixPipeline& pipeline, OptixShaderBindingTable& sbt, OptixTraversableHandle& scene,
                         TransformMatrix lidarPose, TransformMatrix rosTransform, float range)
    {
        currentJob = LaunchLidarParams {
            .range = range,
            .rayCount = dRayPoses.getElemCount(),
            .lidarPose = lidarPose,
            .rosTransform = rosTransform,
            .traversable = scene,
            .lidarNoiseParams = lidarNoiseParams,
            .dRayPoses = dRayPoses.readDevice(),
            .dUnityVisualisationPoints = dUnityVisualisationPoints.writeDevice(),
            .dRosXYZ = dRosXYZ.writeDevice(),
            .dWasHit = dWasHit.writeDevice(),
            .dRandomizationStates = dRandomizationStates.writeDevice()
        };

        dCurrentJob.copyFromHostAsync(&currentJob.value(), 1, stream);

        CUdeviceptr pipelineArgsPtr = dCurrentJob.readDeviceRaw();
        size_t pipelineArgsSize = dCurrentJob.getByteSize();
        dim3 launchDims = {static_cast<unsigned int>(currentJob->rayCount), 1, 1};

        addGaussianNoise(stream, dRayPoses, lidarNoiseParams, dRandomizationStates, dRayPoses);
        OPTIX_CHECK(optixLaunch( pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sbt, launchDims.x, launchDims.y, launchDims.y));

        Point3f lidar_origin_position = {lidarPose[3], lidarPose[7], lidarPose[11]};
        addGaussianNoise(stream, dRosXYZ, dUnityVisualisationPoints, lidar_origin_position, lidarNoiseParams, dRandomizationStates, dRosXYZ, dUnityVisualisationPoints);

        formatPCLsAsync(stream, dWasHit, dHitsBeforeIndex, dRosXYZ, dUnityVisualisationPoints, dLidarArrayRingIds,
                        dDensePoint3f, dDensePCL12, dDensePCL24, dDensePCL48);

        hDensePoint3f.copyFromDeviceAsync(dDensePoint3f, stream);
        hDensePCL12.copyFromDeviceAsync(dDensePCL12, stream);
        hDensePCL24.copyFromDeviceAsync(dDensePCL24, stream);
        hDensePCL48.copyFromDeviceAsync(dDensePCL48, stream);
    }

    int getResultsSize()
    {
        densePointCount = -1;
        CUDA_CHECK(MemcpyAsync(&densePointCount.value(),
                          dHitsBeforeIndex.readDevice() + dHitsBeforeIndex.getElemCount() - 1,
                          sizeof(densePointCount.value()),
                          cudaMemcpyDeviceToHost,
                          stream));
        CUDA_CHECK(StreamSynchronize(stream));
        return densePointCount.value();
    }


    void getResults(int buffersSize,
                    Point3f* outXYZ,
                    PCL12* outPCL12,
                    PCL24* outPCL24,
                    PCL48* outPCL48)
    {
        // densePointCount.has_value() guarantees that the stream has been synchronized
        if (!densePointCount.has_value()) {
            throw std::logic_error("getResults() has been called without a prior call to getResultsSize()");
        }

        if (densePointCount != buffersSize) {
            auto msg = fmt::format("Invalid buffer size ({}) for the size of the requested PCL ({})\n",
                                   buffersSize, densePointCount.value());
            throw std::invalid_argument(msg);
        }

        CUDA_CHECK(StreamSynchronize(stream));
        memcpy(outXYZ, hDensePoint3f.readHost(), hDensePoint3f.getByteSize());
        memcpy(outPCL12, hDensePCL12.readHost(), hDensePCL12.getByteSize());
        memcpy(outPCL24, hDensePCL24.readHost(), hDensePCL24.getByteSize());
        memcpy(outPCL48, hDensePCL48.readHost(), hDensePCL48.getByteSize());

        currentJob.reset();
        densePointCount.reset();
    }

    std::optional<LaunchLidarParams> currentJob;
    DeviceBuffer<LaunchLidarParams> dCurrentJob;
    std::optional<int> densePointCount;

    // GPU INPUT
    DeviceBuffer<TransformMatrix> dRayPoses {"dRayPoses"};
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

    LidarNoiseParams lidarNoiseParams;
};
