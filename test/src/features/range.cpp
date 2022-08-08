#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/experimental.h>
#include <utils/testUtils.h>

using namespace ::testing;

TEST(Lidar, range)
{
    rgl_mesh_t cube_mesh;
    rgl_entity_t cube;
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
    int hitpointCount = 0;
    rgl_vec3f results[1] = { 0 };

    EXPECT_EQ(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length), rgl_status_t::RGL_SUCCESS);
    EXPECT_EQ(rgl_entity_create(&cube, nullptr, cube_mesh), rgl_status_t::RGL_SUCCESS);
    EXPECT_EQ(rgl_entity_set_pose(cube, &entity_tf), rgl_status_t::RGL_SUCCESS);
    EXPECT_EQ(rgl_lidar_create(&lidar, &ray_tf, 1), rgl_status_t::RGL_SUCCESS);

    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, 1);
    ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);

    EXPECT_EQ(rgl_lidar_set_range(lidar, 4.5f), rgl_status_t::RGL_SUCCESS);

    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, 1);
    ASSERT_FLOAT_EQ(results[0].value[2], 4.0f);

    EXPECT_EQ(rgl_lidar_set_range(lidar, 3.5f), rgl_status_t::RGL_SUCCESS);

    getLidarResults(lidar, &hitpointCount, results);

    ASSERT_EQ(hitpointCount, 0);

    rgl_mesh_destroy(cube_mesh);
    rgl_entity_destroy(cube);
    rgl_lidar_destroy(lidar);
}
