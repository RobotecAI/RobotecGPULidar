// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <optix.h>
#include <RGLFields.hpp>

struct RaytraceRequestContext
{
	// Input
	Vec3f sensorLinearVelocityXYZ;
	Vec3f sensorAngularVelocityRPY;
	bool doApplyDistortion;

	const Mat3x4f* rays;
	size_t rayCount;

	Mat3x4f rayOriginToWorld;

	const Vec2f* rayRanges;
	size_t rayRangesCount;

	const int* ringIds;
	size_t ringIdsCount;

	const float* rayTimeOffsets;
	size_t rayTimeOffsetsCount;

	OptixTraversableHandle scene;
	double sceneTime;
	float sceneDeltaTime;

	// Output
	Field<XYZ_VEC3_F32>::type* xyz;
	Field<IS_HIT_I32>::type* isHit;
	Field<RAY_IDX_U32>::type* rayIdx;
	Field<RING_ID_U16>::type* ringIdx;
	Field<DISTANCE_F32>::type* distance;
	Field<INTENSITY_F32>::type* intensity;
	Field<TIME_STAMP_F64>::type* timestamp;
	Field<ENTITY_ID_I32>::type* entityId;
	Field<ABSOLUTE_VELOCITY_VEC3_F32>::type* pointAbsVelocity;
};
static_assert(std::is_trivially_copyable<RaytraceRequestContext>::value);
