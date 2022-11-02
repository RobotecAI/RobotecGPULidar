#pragma once

#include <cuda_runtime_api.h>

#include <spdlog/fmt/fmt.h>

static inline void onCUDAError() {}

#define CHECK_CUDA(call)                                                                            \
do                                                                                                  \
{                                                                                                   \
    cudaError_t status = call;                                                                      \
    if (status != cudaSuccess) {                                                                    \
        auto message = fmt::format("cuda error: {} (code={}) @ {}:{}",                              \
        cudaGetErrorString(status), status, __FILE__, __LINE__);                                    \
        onCUDAError();                                                                              \
        throw std::runtime_error(message);                                                          \
    }                                                                                               \
}                                                                                                   \
while(false)

#define CHECK_CUDA_NO_THROW(call)                                                           \
do                                                                                          \
{                                                                                           \
    cudaError_t status = call;                                                              \
    if (status != cudaSuccess) {                                                            \
        fmt::print(stderr, "cuda error: {} (code={})", cudaGetErrorString(status), status); \
        std::exit(EXIT_FAILURE);                                                            \
    }                                                                                       \
}                                                                                           \
while(false)

#define HostDevFn __host__ __device__
#define DevFn __device__
#define HostFn __host__
