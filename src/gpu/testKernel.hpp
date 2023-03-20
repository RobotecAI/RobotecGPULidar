#pragma once

#include <unordered_map>
#include <memory>

#include <rgl/api/core.h>
#include <gpu/GPUFieldDesc.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>

void testKernelWrapper(size_t pointCount, float* inFloats, cudaStream_t stream);

