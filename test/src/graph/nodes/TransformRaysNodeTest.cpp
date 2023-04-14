#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

class TransformRaysNodeTest : public RGLTest{
protected:
    rgl_mat3x4f identity = Mat3x4f::identity().toRGL();
};

TEST_F(TransformRaysNodeTest, invalid_arguments)
{
    rgl_node_t transformRaysNode = nullptr;

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, &identity), "node != nullptr");

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(&transformRaysNode, nullptr), "transform != nullptr");
}

TEST_F(TransformRaysNodeTest, valid_arguments)
{
    rgl_node_t transformRaysNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRaysNode, &identity));
    ASSERT_THAT(transformRaysNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRaysNode, &identity));
}
