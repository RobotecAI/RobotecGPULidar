#include <utils.hpp>

class FromMat3x4fRaysNodeTest : public RGLTest{
protected:
    rgl_node_t raysFromMatNode;

    FromMat3x4fRaysNodeTest()
    {
        raysFromMatNode = nullptr;
    }
};

TEST_F(FromMat3x4fRaysNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(nullptr, &identityTestTransform, sizeof(identityTestTransform)), "node != nullptr");
}

TEST_F(FromMat3x4fRaysNodeTest, invalid_argument_rays)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&raysFromMatNode, nullptr, sizeof(identityTestTransform)), "rays != nullptr");
}

TEST_F(FromMat3x4fRaysNodeTest, invalid_argument_ray_count)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identityTestTransform, 0), "ray_count > 0");
}

TEST_F(FromMat3x4fRaysNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identityTestTransform, sizeof(identityTestTransform)));
    ASSERT_THAT(raysFromMatNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identityTestTransform, sizeof(identityTestTransform)));
}
