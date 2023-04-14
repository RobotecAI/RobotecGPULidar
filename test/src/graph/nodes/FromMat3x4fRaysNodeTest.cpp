#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

class FromMat3x4fRaysNodeTest : public RGLTest{
protected:
    rgl_mat3x4f identity = Mat3x4f::identity().toRGL();
};

TEST_F(FromMat3x4fRaysNodeTest, invalid_arguments)
{
    rgl_node_t raysFromMatNode = nullptr;

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(nullptr, &identity, sizeof(identity)), "node != nullptr");

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&raysFromMatNode, nullptr, sizeof(identity)), "rays != nullptr");

    raysFromMatNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identity, 0), "ray_count > 0");
}

TEST_F(FromMat3x4fRaysNodeTest, valid_arguments)
{
    rgl_node_t raysFromMatNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identity, sizeof(identity)));
    ASSERT_THAT(raysFromMatNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identity, sizeof(identity)));
}
