#include <utils.hpp>

class TransformRaysNodeTest : public RGLTest{
protected:
    rgl_node_t transformRaysNode;

    TransformRaysNodeTest()
    {
        transformRaysNode = nullptr;
    }
};

TEST_F(TransformRaysNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, &identityTestTransform), "node != nullptr");
}

TEST_F(TransformRaysNodeTest, invalid_argument_transform)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(&transformRaysNode, nullptr), "transform != nullptr");
}

TEST_F(TransformRaysNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRaysNode, &identityTestTransform));
    ASSERT_THAT(transformRaysNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRaysNode, &identityTestTransform));
}
