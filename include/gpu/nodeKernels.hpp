#pragma once

#include <unordered_map>
#include <memory>

#include <rgl/api/experimental.h>
#include <gpu/GPUFieldDesc.hpp>
#include <math/Mat3x4f.hpp>

/*
 * The following functions are asynchronous!
 */

void gpuFormat(cudaStream_t, size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc *fields, char *out);
void gpuTransformRays(cudaStream_t, size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform);
void gpuFindCompaction(cudaStream_t stream, size_t pointCount, const bool* isHit, int32_t* hitCountInclusive, size_t* outHitCount);
void gpuApplyCompaction(cudaStream_t stream, size_t pointCount, size_t fieldSize, const bool* shouldWrite, const int32_t* writeIndex, char* dst, const char* src);