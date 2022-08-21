#pragma once

#include <unordered_map>
#include <memory>

#include <rgl/api/experimental.h>
#include <gpu/GPUFieldDesc.hpp>
#include <VArray.hpp>

/*
 * The following functions are asynchronous!
 */
void gpuFormat(size_t fieldCount, GPUFieldDesc* fields, size_t pointCount, size_t pointSize, char* out);
