#pragma once

#include "formatPCL.h"

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
        // TODO: register this to avoid double free due to C# LidarRenderer/finalize LidarContext/finalize
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
    }

    // TODO: Arguments such as OptiX Pipeline and SBT should be passed coupled (as structure)
    void scheduleRaycast(OptixPipeline& pipeline, OptixShaderBindingTable& sbt, OptixTraversableHandle& scene, TransformMatrix lidarPose, TransformMatrix rosTransform, float range)
    {
        currentJob = LaunchLidarParams {
            .range = range,
            .rayCount = dRayPoses.getElemCount(),
            .lidarPose = lidarPose,
            .rosTransform = rosTransform,
            .traversable = scene,
            .dRayPoses = dRayPoses.readDevice(),
            .dUnityVisualisationPoints = dUnityVisualisationPoints.writeDevice(),
            .dRosXYZ = dRosXYZ.writeDevice(),
            .dWasHit = dWasHit.writeDevice(),
        };

        dCurrentJob.copyFromHostAsync(&currentJob.value(), 1, stream);

        CUdeviceptr pipelineArgsPtr = dCurrentJob.readDeviceRaw();
        size_t pipelineArgsSize = dCurrentJob.getByteSize();
        dim3 launchDims = {static_cast<unsigned int>(currentJob->rayCount), 1, 1};
        OPTIX_CHECK(optixLaunch( pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sbt, launchDims.x, launchDims.y, launchDims.y));

        formatPCLsAsync(stream, dWasHit, dHitsBeforeIndex, dRosXYZ, dUnityVisualisationPoints, dLidarArrayRingIds,
                        dDensePoint3f, dDensePCL12, dDensePCL24, dDensePCL48);
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

        // TODO: here lies a potential performance improvement (but needs profiling):
        // - Copy (device -> paged host [aka malloc-ed]) is indirect (buffered) and therefore slower
        //   - Alternative approach would be to schedule earlier async copy device -> pinned host [aka cudaMallocHost-ed],
        //     and getResults would do a memcpy (pinned -> paged), which is likely to be faster (~40GiB/s) than (device -> paged host) (?? GiB/s)
        // - These copies can be done in parallel (either in different streams or threads, depending on the point above
        dDensePoint3f.copyPrefixToHostAsync(outXYZ, buffersSize, stream);
        dDensePCL12.copyPrefixToHostAsync(outPCL12, buffersSize, stream);
        dDensePCL24.copyPrefixToHostAsync(outPCL24, buffersSize, stream);
        dDensePCL48.copyPrefixToHostAsync(outPCL48, buffersSize, stream);

        CUDA_CHECK(StreamSynchronize(stream));

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
};
