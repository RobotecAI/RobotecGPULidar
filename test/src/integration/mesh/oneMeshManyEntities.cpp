#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/experimental.h>
#include <testModels.hpp>
#include <utils/testUtils.h>

using namespace ::testing;

class OneMeshManyEntities : public ::testing::Test {
protected:
    void SetUp() override
    {
        rgl_lidar_create(&lidar, &ray_tf, 1);
    }

    rgl_mesh_t cube_mesh;
    rgl_entity_t entity;
    rgl_lidar_t lidar;

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

TEST_F(OneMeshManyEntities, one_mesh)
{
    EXPECT_EQ(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length), rgl_status_t::RGL_SUCCESS);
}

TEST_F(OneMeshManyEntities, delete_mesh)
{
    EXPECT_RGL_SUCCESS(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length));
    EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, cube_mesh));
    EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf));

    EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
    EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpointCount));
    EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

    EXPECT_EQ(hitpointCount, 1);
    EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);

    EXPECT_RGL_SUCCESS(rgl_mesh_destroy(cube_mesh)); // shouldn't be possible?
    EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entity_tf)); // ?

    EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
    EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpointCount));
    EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, results));

    EXPECT_EQ(hitpointCount, 1);
    EXPECT_FLOAT_EQ(results[0].value[2], 4.0f);
}
