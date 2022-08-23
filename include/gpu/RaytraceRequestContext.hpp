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
	RGLField<RGL_FIELD_XYZ_F32>::Type* xyz;
	RGLField<RGL_FIELD_IS_HIT_I32>::Type* isHit;
};
