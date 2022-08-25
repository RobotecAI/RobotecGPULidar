#include <cuda_runtime.h>
#include <optix_device.h>

#include <math/Vector.hpp>
#include <math/Mat3x4f.hpp>
#include <cassert>

#include <gpu/RaytraceRequestContext.hpp>
#include <gpu/ShaderBindingTableTypes.h>

#define HOSTDEVICE __device__
// #include "linearGeometry.h"

extern "C" static __constant__ RaytraceRequestContext ctx;

extern "C" __global__ void __raygen__()
{
	Mat3x4f ray = ctx.rays[optixGetLaunchIndex().x];

	Vec3f origin = ray * Vec3f{0, 0, 0};
	Vec3f dir = ray * Vec3f{0, 0, 1} - origin;

	unsigned int flags = OPTIX_RAY_FLAG_DISABLE_ANYHIT;
	optixTrace(ctx.scene, origin, dir, 0.0f, ctx.rayRange, 0.0f, OptixVisibilityMask(255), flags, 0, 1, 0);
}

extern "C" __global__ void __closesthit__()
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

	const int rayIdx = optixGetLaunchIndex().x;
	Vec3f hitObject = Vec3f((1 - u - v) * A + u * B + v * C);
	Vec3f hitWorld = optixTransformPointFromObjectToWorldSpace(hitObject);

	// printf("UUUUUU %p\n", (void*) ctx.isHit);
	if (ctx.xyz != nullptr) {
		ctx.xyz[rayIdx] = Vec3f(hitWorld.x(), hitWorld.y(), hitWorld.z());
	}
	if (ctx.isHit != nullptr) {
		ctx.isHit[rayIdx] = true;
	}
}

extern "C" __global__ void __miss__()
{
	// TODO fill XYZ with signed infinities
	const int rayIdx = optixGetLaunchIndex().x;
	if (ctx.isHit != nullptr) {
		ctx.isHit[rayIdx] = false;
	}
}

extern "C" __global__ void __anyhit__(){}
