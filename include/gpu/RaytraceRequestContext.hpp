#pragma once

#include <optix.h>
#include <RGLFields.hpp>

struct RaytraceRequestContext
{
	// Input
	const Mat3x4f* rays;
	size_t rayCount;

	Mat3x4f rayOriginToWorld;
	float rayRange;
	OptixTraversableHandle scene;

	// Output
	Field<XYZ_F32>::type* xyz;
	Field<IS_HIT_I32>::type* isHit;
};
