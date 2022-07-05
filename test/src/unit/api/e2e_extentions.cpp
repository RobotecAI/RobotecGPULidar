#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/e2e_extensions.h>
#include <utils/testUtils.h>

using namespace ::testing;

class ApiE2EExtensionUnitTest : public ::testing::Test {
protected:
    rgl_lidar_t lidar = nullptr;
    rgl_mat3x4f identity = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 0 },
        }
    };
};

TEST_F(ApiE2EExtensionUnitTest, rgl_lidar_set_ring_indices)
{
    int ring_indices[1] = { 0 };

    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_ring_indices(lidar, nullptr, 0), "lidar != nullptr");
    EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &identity, 1));
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_ring_indices(lidar, nullptr, 0), "ring_ids != nullptr");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_ring_indices(lidar, ring_indices, 0), "ring_ids_count > 0");
    EXPECT_RGL_SUCCESS(rgl_lidar_set_ring_indices(lidar, ring_indices, 1));
}

TEST_F(ApiE2EExtensionUnitTest, rgl_lidar_set_gaussian_noise_params)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, 0.0, 0.0, 0.0), "lidar != nullptr");
    EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &identity, 1));
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, -1.0, -1.0, -1.0, -1.0, -1.0), "angular_noise_stddev >= 0.0");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 1.0, 1.0, -1.0, -1.0, -1.0), "distance_noise_stddev_base >= 0.0");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 1.0, 1.0, 1.0, -1.0, -1.0), "distance_noise_stddev_rise_per_meter >= 0.0");
    EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 1.0, 1.0, 1.0, 1.0, 1.0));
}

TEST_F(ApiE2EExtensionUnitTest, rgl_lidar_set_post_raycast_transform)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_post_raycast_transform(lidar, nullptr), "lidar != nullptr");
    EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &identity, 1));
    EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_post_raycast_transform(lidar, nullptr), "transform != nullptr");
    EXPECT_RGL_SUCCESS(rgl_lidar_set_post_raycast_transform(lidar, &identity));
}
