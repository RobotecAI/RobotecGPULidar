#include <cuda_runtime.h>
#include <optix_device.h>

#include <math/Vector.hpp>
#include <math/Mat3x4f.hpp>
#include <cassert>

#include <gpu/RaytraceRequestParams.hpp>
#include <gpu/ShaderBindingTableTypes.h>

#define HOSTDEVICE __device__
// #include "linearGeometry.h"

extern "C" static __constant__ RaytraceRequestContext ctx;

static __forceinline__ __device__ Vec3f forwardVec() {return Vec3f(0.0f, 0.0f, 1.0f); }

extern "C" __global__ void __raygen__()
{
	Mat3x4f rayLocal = ctx.rays[optixGetLaunchIndex().x];
	Mat3x4f rayWorld = ctx.rayOriginToWorld * rayLocal;

	Vec3f origin = rayWorld.translation();
	Vec3f dir = rayWorld.rotation() * forwardVec();

	unsigned int flags = OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT;
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
	Vec3f prd = Vec3f((1 - u - v) * A + u * B + v * C);
	if (ctx.xyz != nullptr) {
		ctx.xyz[rayIdx] = Vec3f(prd.x(), prd.y(), prd.z());
	}

	// Vec3f unityPoint = optixTransformPointFromObjectToWorldSpace(rayHitPoint);
	// Vec3f rosPoint = args.rosTransform * unityPoint;


	// args.dUnityVisualisationPoints[ix].x = unityPoint.x();
	// args.dUnityVisualisationPoints[ix].y = unityPoint.y();
	// args.dUnityVisualisationPoints[ix].z = unityPoint.z();
	// args.dRosXYZ[ix].x = rosPoint.x();
	// args.dRosXYZ[ix].y = rosPoint.y();
	// args.dRosXYZ[ix].z = rosPoint.z();

	// args.dWasHit[optixGetLaunchIndex().x] = 1;
}

extern "C" __global__ void __miss__()
{
	// ctx.dWasHit[optixGetLaunchIndex().x] = 0;
}

extern "C" __global__ void __anyhit__(){}
