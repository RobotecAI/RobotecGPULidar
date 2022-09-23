#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/experimental.h>
#include <utils/testUtils.h>
#include <math/Vector.hpp>

using namespace ::testing;

class Record : public ::testing::Test {
protected:
    void SetUp() override
    {
        EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, &ray_tf, 1));
    }

    void TearDown() override
    {
        EXPECT_RGL_SUCCESS(rgl_lidar_destroy(lidar));
    }

    rgl_mesh_t cube_mesh = nullptr;
    rgl_entity_t entity = nullptr;
    rgl_lidar_t lidar = nullptr;

    rgl_mat3x4f entity_tf = {
            .value = {
                    { 1, 0, 0, 0 },
                    { 0, 1, 0, 0 },
                    { 0, 0, 1, 5 } }
    };
    rgl_mat3x4f ray_tf = {
            .value = {
                    { 1, 0, 0, 0 },
                    { 0, 1, 0, 0 },
                    { 0, 0, 1, 0 },
            }
    };

    rgl_vec3f results[1];
    int hitpointCount = 0;
};

TEST_F(Record, DeleteMesh)
{
    EXPECT_RGL_SUCCESS(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
    EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, cube_mesh));
    EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf));
    getLidarResults(lidar, &hitpointCount, results);

    EXPECT_EQ(hitpointCount, 1);
    EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);

    EXPECT_RGL_SUCCESS(rgl_mesh_destroy(cube_mesh));
    EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf));

    getLidarResults(lidar, &hitpointCount, results);

    EXPECT_EQ(hitpointCount, 1);
    EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);

    EXPECT_RGL_SUCCESS(rgl_entity_destroy(entity));
    getLidarResults(lidar, &hitpointCount, results);

    EXPECT_EQ(hitpointCount, 0);
}

