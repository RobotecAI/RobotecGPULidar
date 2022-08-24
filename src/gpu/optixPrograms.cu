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

static __forceinline__ __device__ Vec3f forwardVec() {return Vec3f(0.0f, 0.0f, 1.0f); }

extern "C" __global__ void __raygen__()
{
	Mat3x4f ray = ctx.rays[optixGetLaunchIndex().x];

	Vec3f tr = ray.translation();
	// Vec3f origin = ray * Vec3f{0, 0, 0};
	// Vec3f dir = ray * Vec3f{0, 0, 1};

	Vec3f origin = ray.translation();
	Vec3f zero = Vec3f(0.0f, 0.0f, 0.0f);
	Vec3f forward = Vec3f(0.0f, 0.0f, 1.0f);
	Vec3f zero_moved = ray * zero;
	Vec3f forward_moved = ray * forward;
	Vec3f dir = Vec3f(forward_moved.x() - zero_moved.x(), forward_moved.y() - zero_moved.y(), forward_moved.z() - zero_moved.z()) ;

	if (optixGetLaunchIndex().x % 12345 == 0) {
		printf("mat:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
			   ray.rc[0][0], ray.rc[0][1], ray.rc[0][2], ray.rc[0][3],
			   ray.rc[1][0], ray.rc[1][1], ray.rc[1][2], ray.rc[1][3],
			   ray.rc[2][0], ray.rc[2][1], ray.rc[2][2], ray.rc[2][3]);
	}
	// printf("origin: %f %f %f \t->\t %f %f %f\n", origin.x(), origin.y(), origin.z(), dir.x(), dir.y(), dir.z());

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

	if (ctx.xyz != nullptr) {
		ctx.xyz[rayIdx] = Vec3f(hitWorld.x(), hitWorld.y(), hitWorld.z());
	}
	if (ctx.isHit != nullptr) {
		ctx.isHit[rayIdx] = true;
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
	const int rayIdx = optixGetLaunchIndex().x;
	if (ctx.isHit != nullptr) {
		ctx.isHit[rayIdx] = false;
	}
}

extern "C" __global__ void __anyhit__(){}
