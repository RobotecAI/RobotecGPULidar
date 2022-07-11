#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/experimental.h>
#include <testModels.hpp>
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
        rgl_mesh_destroy(cube_mesh);
        rgl_entity_destroy(entity);
        rgl_lidar_destroy(lidar);
    }

    void readResults()
    {
        EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
        EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpointCount));
        EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));
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
    std::vector<float> angles = { -30.0f, -20.0f, -10.0f, 10.0f, 20.0f, 30.0f };
    std::vector<float> distances = { -1.5f, -1.0f, 1.0f, 1.5f };
    float expected_distance_z = 3.0f;
};

TEST_F(Transforms, lidar_rotate_x)
{
    for (auto angle_deg : angles) {
        float angle = toRadians(angle_deg);
        float y = expected_distance_z * -std::tan(angle);

        rgl_mat3x4f tf_rotate_x = getTfX(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_x);

        readResults();
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[1], y);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, lidar_rotate_y)
{
    for (auto angle_deg : angles) {
        float angle = toRadians(angle_deg);
        float x = expected_distance_z * std::tan(angle);

        rgl_mat3x4f tf_rotate_y = getTfY(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_y);

        readResults();
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_FLOAT_EQ(results[0].value[0], x);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, lidar_rotate_z)
{
    for (auto angle_deg : angles) {
        float angle = toRadians(angle_deg);

        rgl_mat3x4f tf_rotate_z = getTfZ(0.0, 0.0, 0.0, angle);

        rgl_lidar_set_pose(lidar, &tf_rotate_z);

        readResults();
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, entity_move_x)
{
    for (auto distance : distances) {
        rgl_mat3x4f tf_entity_moved = {
            .value = {
                { 1, 0, 0, distance },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 5 } }
        };

        rgl_entity_set_pose(entity, &tf_entity_moved);

        readResults();
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, entity_move_y)
{
    for (auto distance : distances) {
        rgl_mat3x4f tf_entity_moved = {
            .value = {
                { 1, 0, 0, 0 },
                { 0, 1, 0, distance },
                { 0, 0, 1, 5 } }
        };

        rgl_entity_set_pose(entity, &tf_entity_moved);

        readResults();
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z);
    }
}

TEST_F(Transforms, entity_move_z)
{
    for (auto distance : distances) {
        rgl_mat3x4f tf_entity_moved = {
            .value = {
                { 1, 0, 0, 0 },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 5 + distance } }
        };

        rgl_entity_set_pose(entity, &tf_entity_moved);

        readResults();
        EXPECT_EQ(hitpointCount, 1);
        EXPECT_NEAR(results[0].value[0], 0.0f, error);
        EXPECT_NEAR(results[0].value[1], 0.0f, error);
        EXPECT_FLOAT_EQ(results[0].value[2], expected_distance_z + distance);
    }
}
