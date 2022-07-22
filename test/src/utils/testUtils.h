#pragma once

#include <rgl/api/experimental.h>
#include <vector>
#include <cmath>

#define EXPECT_RGL_SUCCESS(status) EXPECT_EQ(status, rgl_status_t::RGL_SUCCESS)
#define EXPECT_RGL_INVALID_ARGUMENT(status, error_string_msg)     \
    {                                                             \
        EXPECT_EQ(status, rgl_status_t::RGL_INVALID_ARGUMENT);    \
        const char* error_string;                                 \
        rgl_get_last_error_string(&error_string);                 \
        EXPECT_THAT(error_string, HasSubstr("Invalid argument")); \
        EXPECT_THAT(error_string, HasSubstr(error_string_msg));   \
    }

template <typename T>
std::vector<float> computeDistances(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = sqrt(data[i].value[0] * data[i].value[0] + data[i].value[1] * data[i].value[1] + data[i].value[2] * data[i].value[2]);
    }
    return return_data;
}

template <typename T>
std::vector<float> computeAngles(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = std::atan2(data[i].value[2], data[i].value[0]);
    }
    return return_data;
}

void getLidarResults(rgl_lidar_t lidar, int* hitpointCount, void* results);
