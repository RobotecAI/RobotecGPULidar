#pragma once

struct RaytraceRequestContext
{
	Mat3x4f* rays;
	size_t rayCount;

	Mat3x4f rayOriginToWorld;

	float rayRange;

	OptixTraversableHandle scene;
};
