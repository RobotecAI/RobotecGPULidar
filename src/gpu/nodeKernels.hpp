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

#include <unordered_map>
#include <memory>

#include <rgl/api/core.h>
#include <gpu/GPUFieldDesc.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>
#include <thrust/complex.h>
#include <gpu/MultiReturn.hpp>

/*
 * The following functions are asynchronous!
 */

// This could be defined in CompactNode, however such include here causes mess because nvcc does not support C++20.
using CompactionIndexType = int32_t;

void gpuFindCompaction(cudaStream_t, size_t pointCount, const int32_t* shouldCompact, CompactionIndexType* hitCountInclusive,
                       size_t* outHitCount);
void gpuFormatSoaToAos(cudaStream_t, size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc* soaInData,
                       char* aosOutData);
void gpuFormatAosToSoa(cudaStream_t, size_t pointCount, size_t pointSize, size_t fieldCount, const char* aosInData,
                       const GPUFieldDesc* soaOutData);
void gpuTransformRays(cudaStream_t, size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform);
void gpuApplyCompaction(cudaStream_t, size_t pointCount, size_t fieldSize, const int* shouldWrite,
                        const CompactionIndexType* writeIndex, char* dst, const char* src);
void gpuTransformPoints(cudaStream_t, size_t pointCount, const Field<XYZ_VEC3_F32>::type* inPoints,
                        Field<XYZ_VEC3_F32>::type* outPoints, Mat3x4f transform);
void gpuCutField(cudaStream_t, size_t pointCount, char* dst, const char* src, size_t offset, size_t stride, size_t fieldSize);
void gpuFilter(cudaStream_t, size_t count, const Field<RAY_IDX_U32>::type* indices, char* dst, const char* src,
               size_t fieldSize);
void gpuFilterGroundPoints(cudaStream_t stream, size_t pointCount, const Vec3f sensor_up_axis, float ground_angle_threshold,
                           const Field<XYZ_VEC3_F32>::type* inPoints, const Field<NORMAL_VEC3_F32>::type* inNormalsPtr,
                           Field<IS_GROUND_I32>::type* outNonGround, Mat3x4f lidarTransform);
void gpuRadarComputeEnergy(cudaStream_t stream, size_t count, float rayAzimuthStepRad, float rayElevationStepRad, float freq,
                           Mat3x4f lookAtOriginTransform, const Field<RAY_POSE_MAT3x4_F32>::type* rayPose,
                           const Field<DISTANCE_F32>::type* hitDist, const Field<NORMAL_VEC3_F32>::type* hitNorm,
                           const Field<XYZ_VEC3_F32>::type* hitPos, Vector<3, thrust::complex<float>>* outBUBRFactor);
void gpuProcessBeamSamplesFirstLast(cudaStream_t stream, size_t beamCount, int samplesPerBeam, MultiReturnPointers beamSamples,
                                    MultiReturnPointers first, MultiReturnPointers last, const Mat3x4f* beamWorld);