#pragma once

// optix 7
#include <cuda_runtime.h>
#include <optix.h>
#include <sstream>

#include <DisableWarningUtil.h>
DISABLE_WARNING_PUSH
DISABLE_WARNING_MISSING_FIELD_INITIALIZERS
#include <optix_stubs.h>
DISABLE_WARNING_POP

#define CUDA_CHECK(call)                                   \
    do {                                                   \
        cudaError_t rc = cuda##call;                       \
        if (rc != cudaSuccess) {                           \
            std::stringstream txt;                         \
            cudaError_t err = rc; /*cudaGetLastError();*/  \
            txt << "CUDA Error " << cudaGetErrorName(err)  \
                << " (" << cudaGetErrorString(err) << ")"; \
            throw std::runtime_error(txt.str());           \
        }                                                  \
    } while (0)

#define CUDA_CHECK_NOEXCEPT(call) \
    {                             \
        cuda##call;               \
    }

#define OPTIX_CHECK(call)                                                                             \
    do {                                                                                              \
        OptixResult res = call;                                                                       \
        if (res != OPTIX_SUCCESS) {                                                                   \
            fprintf(stderr, "Optix call (%s) failed with code %d (line %d)\n", #call, res, __LINE__); \
            throw std::runtime_error(optixGetErrorName(res));                                         \
        }                                                                                             \
    } while (0)

#define CUDA_SYNC_CHECK()                                                                                \
    do {                                                                                                 \
        cudaStreamSynchronize(0);                                                                        \
        cudaError_t error = cudaGetLastError();                                                          \
        if (error != cudaSuccess) {                                                                      \
            fprintf(stderr, "error (%s: line %d): %s\n", __FILE__, __LINE__, cudaGetErrorString(error)); \
            throw std::runtime_error(cudaGetErrorString(error));                                         \
        }                                                                                                \
    } while (0)
