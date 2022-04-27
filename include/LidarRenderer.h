#pragma once

#include <Optix.hpp>

#include "../common/Texture.h"
#include "LaunchParams.h"
#include "TransformMatrix.h"
#include "data_types/ShaderBindingTableTypes.h"
#include "gdt/utils/CUDABuffer.h"

#include "Logging.h"

#include "DeviceBuffer.hpp"
#include "HostPinnedBuffer.hpp"
#include "data_types/PCLFormats.h"
#include "data_types/LidarSource.h"
#include "LidarContext.hpp"

#include <cstring>

#include <thrust/device_vector.h>
#include <thread>


struct LidarRenderer
{
    // TODO: return reference, static method
    static LidarRenderer& instance();

    void renderCtx(LidarContext* ctx, TransformMatrix lidarPose, float range) { renderCtx(ctx, lidarPose, TransformMatrix::identity(), range); }
    void renderCtx(LidarContext* ctx, TransformMatrix lidarPose, TransformMatrix rosTransform, float range);
    int getResultPointCloudSizeCtx(LidarContext* ctx);
    void downloadPointsCtx(LidarContext* ctx, int maxPointCount, Point3f* outXYZ, PCL12* outPCL12, PCL24* outPCL24, PCL48* outPCL48);
    void downloadPointsCtx(LidarContext* ctx, int maxPointCount, Point3f* outXYZ);

private:
    LidarRenderer();
    ~LidarRenderer();
    std::string getCurrentDeviceName();
};
