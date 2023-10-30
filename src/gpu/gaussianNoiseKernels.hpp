// Copyright 2023 Robotec.AI
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

#include <rgl/api/core.h>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>

void gpuSetupGaussianNoiseGenerator(cudaStream_t stream, size_t pointCount, unsigned int seed,
                                    curandStatePhilox4_32_10_t* outPHILOXStates);
void gpuAddGaussianNoiseAngularRay(cudaStream_t stream, size_t rayCount, float mean, float stDev, rgl_axis_t rotationAxis,
                                   Mat3x4f lookAtOriginTransform, curandStatePhilox4_32_10_t* randomStates,
                                   const Mat3x4f* inRays, Mat3x4f* outRays);
void gpuAddGaussianNoiseAngularHitpoint(cudaStream_t stream, size_t pointCount, float mean, float stDev,
                                        rgl_axis_t rotationAxis, Mat3x4f lookAtOriginTransform,
                                        curandStatePhilox4_32_10_t* randomStates, const Field<XYZ_VEC3_F32>::type* inPoints,
                                        Field<XYZ_VEC3_F32>::type* outPoints, Field<DISTANCE_F32>::type* outDistances);
void gpuAddGaussianNoiseDistance(cudaStream_t stream, size_t pointCount, float mean, float stDevBase, float stDevRisePerMeter,
                                 Mat3x4f lookAtOriginTransform, curandStatePhilox4_32_10_t* randomStates,
                                 const Field<XYZ_VEC3_F32>::type* inPoints, Field<XYZ_VEC3_F32>::type* outPoints,
                                 Field<DISTANCE_F32>::type* outDistances);
