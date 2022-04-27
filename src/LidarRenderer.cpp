#include <fmt/color.h>

#include "LidarRenderer.h"
#include "PerfProbe.h"
#include <thread>
#include <formatPCL.h>

#include <scene/Scene.hpp>

// TODO(prybicki): cleanup this file, or even remove it

LidarRenderer::LidarRenderer()
{
    logInfo("[RGL] Running on GPU: {}\n", getCurrentDeviceName());
    logInfo("[RGL] PID: {}\n", getpid());
}

LidarRenderer::~LidarRenderer() {}


void LidarRenderer::renderCtx(LidarContext *ctx, TransformMatrix lidarPose, TransformMatrix rosTransform, float range)
{
    logInfo("[RGL] render()\n");
    if (Scene::defaultInstance()->getObjectCount() == 0) {
        logWarn("[RGL] LidarRender::render called with 0 meshes\n");
        // Temporary hack to make results disappear when there are no meshes on the scene.
        CUDA_CHECK(MemsetAsync(ctx->dHitsBeforeIndex.writeDevice(), 0, ctx->dHitsBeforeIndex.getByteSize(), ctx->stream));
        return;
    }

    auto as = Scene::defaultInstance()->getAS();
    auto sbt = Scene::defaultInstance()->getSBT();
    ctx->scheduleRaycast(Optix::instance().pipeline, sbt, as, lidarPose, rosTransform, range);
}

int LidarRenderer::getResultPointCloudSizeCtx(LidarContext *ctx)
{
    return ctx->getResultsSize();
}

void LidarRenderer::downloadPointsCtx(LidarContext* ctx, int maxPointCount, Point3f* outXYZ, PCL12* outPCL12, PCL24* outPCL24, PCL48* outPCL48)
{
    logInfo("[RGL] downloadPoints()\n");
    ctx->getResults(maxPointCount, outXYZ, outPCL12, outPCL24, outPCL48);
}

void LidarRenderer::downloadPointsCtx(LidarContext* ctx, int maxPointCount, Point3f* outXYZ)
{
    std::vector<PCL12> dummy12(maxPointCount);
    std::vector<PCL24> dummy24(maxPointCount);
    std::vector<PCL48> dummy48(maxPointCount);
    downloadPointsCtx(ctx, maxPointCount, outXYZ, dummy12.data(), dummy24.data(), dummy48.data());
}

std::string LidarRenderer::getCurrentDeviceName()
{
    int currentDevice = -1;
    cudaDeviceProp deviceProperties {};
    CUDA_CHECK(GetDevice(&currentDevice));
    CUDA_CHECK(GetDeviceProperties(&deviceProperties, currentDevice));
    return std::string(deviceProperties.name);
}

LidarRenderer& LidarRenderer::instance()
{
	static LidarRenderer instance;
	return instance;
}
