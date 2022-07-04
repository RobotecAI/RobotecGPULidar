#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rgl/api/experimental.h>
#include <testModels.hpp>

using namespace ::testing;

TEST(OneMeshManyEntities, one_mesh)
{
    rgl_mesh_t cube_mesh;
    rgl_entity_t entity;

    EXPECT_EQ(rgl_mesh_create(&cube_mesh, cube_vertices, cube_vertices_length, cube_indices, cube_indices_length), rgl_status_t::RGL_SUCCESS);
}
