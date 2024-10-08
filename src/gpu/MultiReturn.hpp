// Copyright 2024 Robotec.AI
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

#include <RGLFields.hpp>

#define MULTI_RETURN_BEAM_VERTICES 19
#define MULTI_RETURN_BEAM_LAYERS 3
#define MULTI_RETURN_BEAM_SAMPLES (1 + (MULTI_RETURN_BEAM_VERTICES * MULTI_RETURN_BEAM_LAYERS))

struct MultiReturnSamplesPointers
{
	// XYZ is not stored for samples, because we calculate it alongside center ray direction. For this reason, only distance
	// is necessary.
	Field<IS_HIT_I32>::type* isHit;
	Field<DISTANCE_F32>::type* distance;
	Field<INTENSITY_F32>::type* intensity;
	Field<LASER_RETRO_F32>::type* laserRetro;
	Field<ENTITY_ID_I32>::type* entityId;
	Field<ABSOLUTE_VELOCITY_VEC3_F32>::type* absVelocity;
	Field<RELATIVE_VELOCITY_VEC3_F32>::type* relVelocity;
	Field<RADIAL_SPEED_F32>::type* radialSpeed;
	Field<NORMAL_VEC3_F32>::type* normal;
	Field<INCIDENT_ANGLE_F32>::type* incidentAngle;
};
