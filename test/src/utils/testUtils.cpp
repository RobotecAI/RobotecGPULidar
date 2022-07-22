#include "testUtils.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

void getLidarResults(rgl_lidar_t lidar, int* hitpointCount, void* results)
{
    EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
    EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, hitpointCount));
    EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));
}
