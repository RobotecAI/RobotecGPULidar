#pragma once

#include <rgl/api/experimental.h>

#define CHECK_RGL(call)                                               \
do                                                                    \
{                                                                     \
    rgl_status_t status = call;                                       \
    if (status != RGL_SUCCESS) {                                      \
        const char* errorMsg = nullptr;                               \
        rgl_get_last_error_string(&errorMsg);                         \
        auto message = fmt::format("RGL error: {} (code={}) @ {}:{}", \
        errorMsg, status, __FILE__, __LINE__);                        \
        onCUDAError();                                                \
        throw std::runtime_error(message);                            \
    }                                                                 \
} while (false)
