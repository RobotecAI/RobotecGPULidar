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

#include <cuda_runtime_api.h>

#include <spdlog/fmt/fmt.h>

static inline void onCUDAError() {}

#define CHECK_CUDA(call)                                                                            \
do                                                                                                  \
{                                                                                                   \
    cudaError_t errCode = call;                                                                     \
    if (errCode == cudaErrorCudartUnloading) {                                                      \
        break;                                                                                      \
    }                                                                                               \
    if (errCode != cudaSuccess) {                                                                   \
        auto message = fmt::format("cuda error: {} (code={}) @ {}:{}",                              \
        cudaGetErrorString(errCode), errCode, __FILE__, __LINE__);                                  \
        onCUDAError();                                                                              \
        throw std::runtime_error(message);                                                          \
    }                                                                                               \
}                                                                                                   \
while(false)

#define CHECK_CUDA_NO_THROW(call)                                                   \
do {                                                                                \
    try {                                                                           \
        CHECK_CUDA(call);                                                           \
    }                                                                               \
    catch(std::runtime_error& cudaError) {                                          \
        RGL_ERROR("Detected CUDA error in no-throw context: {}", cudaError.what()); \
    }                                                                               \
}                                                                                   \
while(false)

#define HostDevFn __host__ __device__
#define DevFn __device__
#define HostFn __host__
