#include <utils.hpp>

class GaussianNoiseAngularRayNodeTest : public RGLTest {
protected:
    rgl_node_t gaussianNoiseNode;

    GaussianNoiseAngularRayNodeTest()
    {
        gaussianNoiseNode = nullptr;
    }
};

// TODO(nebraszka): Parameterize the test to take a permutation of the set of all axes.
TEST_F(GaussianNoiseAngularRayNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(nullptr, 0.0f, 0.0f, RGL_AXIS_X), "node != nullptr");
}

TEST_F(GaussianNoiseAngularRayNodeTest, invalid_argument_st_dev)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, -1.0f, RGL_AXIS_X), "st_dev >= 0");
}

TEST_F(GaussianNoiseAngularRayNodeTest, invalid_argument_rotation_axis)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, 0.0f, (rgl_axis_t)4), "rotation_axis");
}

TEST_F(GaussianNoiseAngularRayNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, 0.0f, RGL_AXIS_X));

    // If (*gaussianNoiseNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 2.0f, 2.0f, RGL_AXIS_X));
}
