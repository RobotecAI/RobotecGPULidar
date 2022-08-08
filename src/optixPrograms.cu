#include "LaunchParams.h"
#include "data_types/ShaderBindingTableTypes.h"
#include <cuda_runtime.h>
#include <optix_device.h>
#include <math/Vector.hpp>

#define HOSTDEVICE __device__
#include "linearGeometry.h"


extern "C" static __constant__ LaunchLidarParams optixLaunchLidarParams;

static __forceinline__ __device__ void* unpackPointer(uint32_t i0, uint32_t i1)
{
    const uint64_t uptr = static_cast<uint64_t>(i0) << 32 | i1;
    void* ptr = reinterpret_cast<void*>(uptr);
    return ptr;
}

static __forceinline__ __device__ void packPointer(void* ptr, uint32_t& i0, uint32_t& i1)
{
    const uint64_t uptr = reinterpret_cast<uint64_t>(ptr);
    i0 = uptr >> 32;
    i1 = uptr & 0x00000000ffffffff;
}

template <typename T>
static __forceinline__ __device__ T* getPRD()
{
    const uint32_t u0 = optixGetPayload_0();
    const uint32_t u1 = optixGetPayload_1();
    return reinterpret_cast<T*>(unpackPointer(u0, u1));
}

extern "C" __global__ void __closesthit__lidar()
{
    const TriangleMeshSBTData& sbtData
        = *(const TriangleMeshSBTData*)optixGetSbtDataPointer();

    const int primID = optixGetPrimitiveIndex();
    assert(primID < sbtData.index_count);
    const Vec3i index = sbtData.index[primID];
    const float u = optixGetTriangleBarycentrics().x;
    const float v = optixGetTriangleBarycentrics().y;

    assert(index.x() < sbtData.vertex_count);
    assert(index.y() < sbtData.vertex_count);
    assert(index.z() < sbtData.vertex_count);
    const Vec3f& A = sbtData.vertex[index.x()];
    const Vec3f& B = sbtData.vertex[index.y()];
    const Vec3f& C = sbtData.vertex[index.z()];

    Vec3f& prd = *(Vec3f*)getPRD<Vec3f>();
    prd = Vec3f((1 - u - v) * A + u * B + v * C);

    Vec3f rayHitPoint = Vec3f(prd.x(), prd.y(), prd.z());
    Vec3f unityPoint = optixTransformPointFromObjectToWorldSpace(rayHitPoint);
    Vec3f rosPoint = multiply3x4TransformByVector3(optixLaunchLidarParams.rosTransform, unityPoint);

    const int ix = optixGetLaunchIndex().x;

    optixLaunchLidarParams.dUnityVisualisationPoints[ix].x = unityPoint.x();
    optixLaunchLidarParams.dUnityVisualisationPoints[ix].y = unityPoint.y();
    optixLaunchLidarParams.dUnityVisualisationPoints[ix].z = unityPoint.z();
    optixLaunchLidarParams.dRosXYZ[ix].x = rosPoint.x();
    optixLaunchLidarParams.dRosXYZ[ix].y = rosPoint.y();
    optixLaunchLidarParams.dRosXYZ[ix].z = rosPoint.z();

    optixSetPayload_3(1);
}

extern "C" __global__ void __anyhit__lidar()
{ /*! for this simple example, this will remain empty */
}

extern "C" __global__ void __miss__lidar()
{
    Vec3f& prd = *(Vec3f*)getPRD<Vec3f>();
    prd = Vec3f(0.f);
    optixSetPayload_3(0);
}

extern "C" __global__ void __raygen__renderLidar()
{
    const int ix = optixGetLaunchIndex().x;

    Vec3f lidarPositionPRD = Vec3f(0.f);

    // the values we store the PRD pointer in:
    uint32_t u0 = 0U;
    uint32_t u1 = 0U;
    uint32_t u2 = 0U;
    uint32_t u3 = 0U;
    packPointer(&lidarPositionPRD, u0, u1);

    Mat3x4f ray_pose_local = optixLaunchLidarParams.dRayPoses[ix];
    Mat3x4f ray_pose_global = multiply3x4TransformMatrices(optixLaunchLidarParams.lidarPose, ray_pose_local);

    Vec3f from = getTranslationFrom3x4Transform(ray_pose_global);
    Vec3f zero = Vec3f(0.0f, 0.0f, 0.0f);
    Vec3f forward = Vec3f(0.0f, 0.0f, 1.0f);
    Vec3f zero_moved = multiply3x4TransformByVector3(ray_pose_global, zero);
    Vec3f forward_moved = multiply3x4TransformByVector3(ray_pose_global, forward);
    Vec3f dir = Vec3f(forward_moved.x() - zero_moved.x(), forward_moved.y() - zero_moved.y(), forward_moved.z() - zero_moved.z()) ;

    optixTrace(optixLaunchLidarParams.traversable,
        from,
        dir,
        0.f, // tmin
        optixLaunchLidarParams.range,
        0.0f, // rayTime
        OptixVisibilityMask(255),
        OPTIX_RAY_FLAG_DISABLE_ANYHIT, //OPTIX_RAY_FLAG_NONE,
        LIDAR_RAY_TYPE, // SBT offset
        LIDAR_RAY_TYPE_COUNT, // SBT stride
        LIDAR_RAY_TYPE, // missSBTIndex
        u0, u1, u2, u3);

    if (u3) {
        optixLaunchLidarParams.dWasHit[ix] = 1;
    } else {
        optixLaunchLidarParams.dWasHit[ix] = 0;
    }
}
