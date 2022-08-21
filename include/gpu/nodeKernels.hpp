#pragma once

#include <unordered_map>
#include <memory>

#include <rgl/api/experimental.h>
#include <gpu/GPUFieldDesc.hpp>
#include <math/Mat3x4f.hpp>

/*
 * The following functions are asynchronous!
 */

void gpuFormat(size_t pointCount, size_t pointSize, size_t fieldCount, const GPUFieldDesc *fields, char *out);
void gpuTransformRays(size_t rayCount, const Mat3x4f* inRays, Mat3x4f* outRays, Mat3x4f transform);