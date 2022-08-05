#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/e2e_extensions.h>
#include <rgl/api/apiUnitTests.h>
#include <testModels.hpp>
#include <utils/statistical_utils.h>
#include <utils/testUtils.h>

using namespace ::testing;

class Transforms : public ::testing::Test {
protected:
    void SetUp() override
    {
        rgl_mesh_create(&cube_mesh, cube_vertices_big, cube_vertices_length, cube_indices, cube_indices_length);
        rgl_entity_create(&entity, nullptr, cube_mesh);
        rgl_entity_set_pose(entity, &tf_entity);
        rgl_lidar_create(&lidar, &tf_zero, 1);
    }

    void TearDown() override
    {
        rgl_cleanup();
    }

    float toRadians(float angle_radians)
    {
        return angle_radians * M_PI / 180;
    }

    rgl_mesh_t cube_mesh = nullptr;
    rgl_entity_t entity = nullptr;
    rgl_lidar_t lidar = nullptr;

    rgl_mat3x4f tf_entity = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 5 } }
    };

    rgl_mat3x4f tf_zero = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 0 },
        }
    };

    rgl_mat3x4f getTfX(float x, float y, float z, float angleX)
    {
        return {
            .value = {
                { 1, 0, 0, x },
                { 0, std::cos(angleX), -std::sin(angleX), y },
                { 0, std::sin(angleX), std::cos(angleX), z },
            }
        };
    }

    rgl_mat3x4f getTfY(float x, float y, float z, float angleY)
    {
        return {
            .value = {
                { std::cos(angleY), 0, std::sin(angleY), x },
                { 0, 1, 0, y },
                { -std::sin(angleY), 0, std::cos(angleY), z },
            }
        };
    }

    rgl_mat3x4f getTfZ(float x, float y, float z, float angleZ)
    {
        return {
            .value = {
                { std::cos(angleZ), -std::sin(angleZ), 0, x },
                { std::sin(angleZ), std::cos(angleZ), 0, y },
                { 0, 0, 1, z },
            }
        };
    }

    rgl_vec3f results[1];
    int hitpointCount = 0;
    float error = 10e-6;
    std::vector<float> angles_in_entity = { -30.0f, -20.0f, -10.0f, 10.0f, 20.0f, 30.0f };
    std::vector<float> angles_out_entity = { -60.0f, 60.0f };
    std::vector<float> distances_in_entity = { -1.5f, -1.0f, 1.0f, 1.5f };
    std::vector<float> distances_out_entity = { -3.0f, 3.0f };
    float expected_distance_z = 3.0f;
    int no_of_rays = 100000;
    float distance_std_dev = 0.1;
};

TEST_F(Transforms, LidarRotateX)
{
    for (auto angle_deg : angles_in_entity) {
        float angle = toRadians(angle_deg);
        float y = expected_distance_z * -std::tan(angle);

        rgl_mat3x4f tf_rotate_x = getTfX(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_x);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[1], y);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }

    for (auto angle_deg : angles_out_entity) {
        float angle = toRadians(angle_deg);
        float y = expected_distance_z * -std::tan(angle);

        rgl_mat3x4f tf_rotate_x = getTfX(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_x);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 0);
    }
}

TEST_F(Transforms, LidarRotateY)
{
    for (auto angle_deg : angles_in_entity) {
        float angle = toRadians(angle_deg);
        float x = expected_distance_z * std::tan(angle);

        rgl_mat3x4f tf_rotate_y = getTfY(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_y);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_FLOAT_EQ(results[0].value[0], x);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }

    for (auto angle_deg : angles_out_entity) {
        float angle = toRadians(angle_deg);
        float x = expected_distance_z * std::tan(angle);

        rgl_mat3x4f tf_rotate_y = getTfY(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_y);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 0);
    }
}

TEST_F(Transforms, LidarRotateZ)
{
    for (auto angle_deg : angles_in_entity) {
        float angle = toRadians(angle_deg);

        rgl_mat3x4f tf_rotate_z = getTfZ(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_z);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, EntityMoveX)
{
    for (auto distance : distances_in_entity) {
        rgl_mat3x4f tf_entity_moved = {
            .value = {
                { 1, 0, 0, distance },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 5 } }
        };

        rgl_entity_set_pose(entity, &tf_entity_moved);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, EntityMoveY)
{
    for (auto distance : distances_in_entity) {
        rgl_mat3x4f tf_entity_moved = {
            .value = {
                { 1, 0, 0, 0 },
                { 0, 1, 0, distance },
                { 0, 0, 1, 5 } }
        };

        rgl_entity_set_pose(entity, &tf_entity_moved);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, EntityMoveZ)
{
    for (auto distance : distances_in_entity) {
        rgl_mat3x4f tf_entity_moved = {
            .value = {
                { 1, 0, 0, 0 },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 5.0f + distance } }
        };

        rgl_entity_set_pose(entity, &tf_entity_moved);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z + distance);
    }
}

TEST_F(Transforms, RayMoveZ)
{
    rgl_mat3x4f tf_ray = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 1 },
        }
    };
    rgl_lidar_destroy(lidar);
    rgl_lidar_create(&lidar, &tf_ray, 1);

    getLidarResults(lidar, &hitpointCount, results);
    EXPECT_EQ(hitpointCount, 1);
    EXPECT_NEAR(results[0].value[0], 0.0f, error);
    EXPECT_NEAR(results[0].value[1], 0.0f, error);
    EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
}

TEST_F(Transforms, RayMoveY)
{
    for (auto ray_move_distance_y : distances_out_entity) {
        rgl_mat3x4f tf_ray = {
            .value = {
                { 1, 0, 0, 0 },
                { 0, 1, 0, ray_move_distance_y },
                { 0, 0, 1, 0 },
            }
        };
        rgl_lidar_destroy(lidar);
        rgl_lidar_create(&lidar, &tf_ray, 1);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 0);
    }
}

TEST_F(Transforms, RayMoveX)
{
    for (auto ray_move_distance_x : distances_out_entity) {
        rgl_mat3x4f tf_ray = {
            .value = {
                { 1, 0, 0, ray_move_distance_x },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 0 },
            }
        };
        rgl_lidar_destroy(lidar);
        rgl_lidar_create(&lidar, &tf_ray, 1);

        getLidarResults(lidar, &hitpointCount, results);
        EXPECT_EQ(hitpointCount, 0);
    }
}

TEST_F(Transforms, RayMoveZGaussianNoise)
{
    rgl_mat3x4f tf_ray_moved = {
        .value = {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 1 },
        }
    };
    std::vector<rgl_mat3x4f> tf_rays_moved;
    for (size_t i = 0; i < no_of_rays; ++i) {
        tf_rays_moved.push_back(tf_ray_moved);
    }
    rgl_vec3f results[no_of_rays];

    rgl_lidar_destroy(lidar);
    rgl_lidar_create(&lidar, tf_rays_moved.data(), tf_rays_moved.size());
    EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, distance_std_dev, 0.0, 0.0));

    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, no_of_rays);
    auto distances = computeDistances(results, no_of_rays);
    auto [average_distance, distance_st_dev] = mean_and_stdev(distances);
    EXPECT_NEAR(average_distance, expected_distance_z, 0.001f);
    EXPECT_NEAR(distance_st_dev, distance_std_dev, 0.002f);
}

TEST_F(Transforms, RayMoveYGaussianNoise)
{
    for (auto ray_move_distance_y : distances_out_entity) {
        rgl_mat3x4f tf_ray_moved = {
            .value = {
                { 1, 0, 0, 0 },
                { 0, 1, 0, ray_move_distance_y },
                { 0, 0, 1, 0 },
            }
        };
        std::vector<rgl_mat3x4f> tf_rays_moved;
        for (size_t i = 0; i < no_of_rays; ++i) {
            tf_rays_moved.push_back(tf_ray_moved);
        }
        rgl_vec3f results[no_of_rays];

        rgl_lidar_destroy(lidar);
        rgl_lidar_create(&lidar, tf_rays_moved.data(), tf_rays_moved.size());
        EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, distance_std_dev, 0.0, 0.0));

        getLidarResults(lidar, &hitpointCount, results);

        ASSERT_EQ(hitpointCount, 0);
    }
}

TEST_F(Transforms, RayMoveXGaussianNoise)
{
    for (auto ray_move_distance_X : distances_out_entity) {
        rgl_mat3x4f tf_ray_moved = {
            .value = {
                { 1, 0, 0, ray_move_distance_X },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 0 },
            }
        };
        std::vector<rgl_mat3x4f> tf_rays_moved;
        for (size_t i = 0; i < no_of_rays; ++i) {
            tf_rays_moved.push_back(tf_ray_moved);
        }
        rgl_vec3f results[no_of_rays];

        rgl_lidar_destroy(lidar);
        rgl_lidar_create(&lidar, tf_rays_moved.data(), tf_rays_moved.size());
        EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, rgl_angular_noise_type_t::RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 0.0, 0.0, distance_std_dev, 0.0, 0.0));
        getLidarResults(lidar, &hitpointCount, results);

        ASSERT_EQ(hitpointCount, 0);
    }
}
