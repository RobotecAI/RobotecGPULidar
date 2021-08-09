#include "LaunchParams.h"
#include <cuda_runtime.h>
#include <optix_device.h>

#define NDEBUG
//#undef NDEBUG
#include <assert.h>

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
    const vec3i index = sbtData.index[primID];
    const float u = optixGetTriangleBarycentrics().x;
    const float v = optixGetTriangleBarycentrics().y;

    assert(index.x < sbtData.vertex_count);
    assert(index.y < sbtData.vertex_count);
    assert(index.z < sbtData.vertex_count);
    const vec3f& A = sbtData.vertex[index.x];
    const vec3f& B = sbtData.vertex[index.y];
    const vec3f& C = sbtData.vertex[index.z];

    vec3f& prd = *(vec3f*)getPRD<vec3f>();
    prd = vec3f((1 - u - v) * A + u * B + v * C);

    uint32_t intensity = 0;
    if (sbtData.hasTexture && sbtData.texcoord) {
        assert(index.x < sbtData.texcoord_count);
        assert(index.y < sbtData.texcoord_count);
        assert(index.z < sbtData.texcoord_count);
        const vec2f tc
            = (1.f - u - v) * sbtData.texcoord[index.x]
            + u * sbtData.texcoord[index.y]
            + v * sbtData.texcoord[index.z];

        // Texture access is resilient to out-of-bounds and is governed by cudaTextureAddressMode.
        vec4f fromTexture = tex2D<float4>(sbtData.texture, tc.x, tc.y);
        float intensityFloat = fromTexture.w; // only alpha canal
        memcpy(&intensity, &intensityFloat, sizeof(intensityFloat));
    }

    vec3f rayHitPoint = vec3f(prd.x, prd.y, prd.z);
    vec3f transformedPoint = optixTransformPointFromObjectToWorldSpace(rayHitPoint);

    const int ix = optixGetLaunchIndex().x;

    optixLaunchLidarParams.positionBuffer[ix * 4] = transformedPoint.x;
    optixLaunchLidarParams.positionBuffer[ix * 4 + 1] = transformedPoint.y;
    optixLaunchLidarParams.positionBuffer[ix * 4 + 2] = transformedPoint.z;
    optixLaunchLidarParams.positionBuffer[ix * 4 + 3] = intensity;
    optixSetPayload_3(1);
}

extern "C" __global__ void __anyhit__lidar()
{ /*! for this simple example, this will remain empty */
}

extern "C" __global__ void __miss__lidar()
{
    vec3f& prd = *(vec3f*)getPRD<vec3f>();
    prd = vec3f(0.f);
    optixSetPayload_3(0);
}

extern "C" __global__ void __raygen__renderLidar()
{
    const int ix = optixGetLaunchIndex().x;
    int lidarX = ix;
    int lidarCount = 0;

    while (optixLaunchLidarParams.raysPerLidarBuffer[lidarCount] < lidarX) {
        lidarX -= optixLaunchLidarParams.raysPerLidarBuffer[lidarCount];
        lidarCount++;
    }

    vec3f lidarPositionPRD = vec3f(0.f);

    // the values we store the PRD pointer in:
    uint32_t u0, u1, u2, u3;
    packPointer(&lidarPositionPRD, u0, u1);

    vec3f from = vec3f(optixLaunchLidarParams.sourceBuffer[lidarCount * 3], optixLaunchLidarParams.sourceBuffer[lidarCount * 3 + 1], optixLaunchLidarParams.sourceBuffer[lidarCount * 3 + 2]);
    vec3f dir = vec3f(optixLaunchLidarParams.rayBuffer[ix * 3], optixLaunchLidarParams.rayBuffer[ix * 3 + 1], optixLaunchLidarParams.rayBuffer[ix * 3 + 2]);

    optixTrace(optixLaunchLidarParams.traversable,
        from, // from
        dir, // direction
        0.f, // tmin
        optixLaunchLidarParams.rangeBuffer[lidarCount], // tmax
        0.0f, // rayTime
        OptixVisibilityMask(255),
        OPTIX_RAY_FLAG_DISABLE_ANYHIT, //OPTIX_RAY_FLAG_NONE,
        LIDAR_RAY_TYPE, // SBT offset
        LIDAR_RAY_TYPE_COUNT, // SBT stride
        LIDAR_RAY_TYPE, // missSBTIndex
        u0, u1, u2, u3);

    if (u3) {
        optixLaunchLidarParams.hitBuffer[ix] = 1;
    } else {
        optixLaunchLidarParams.hitBuffer[ix] = 0;
    }
}
