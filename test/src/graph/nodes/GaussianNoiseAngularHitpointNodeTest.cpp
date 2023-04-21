#include <utils.hpp>

class GaussianNoiseAngularHitpointNodeTest : public RGLTest {
protected:
    rgl_node_t gaussianNoiseNode;

    GaussianNoiseAngularHitpointNodeTest()
    {
        gaussianNoiseNode = nullptr;
    }
};

// TODO(nebraszka): Parameterize the test (axes)

TEST_F(GaussianNoiseAngularHitpointNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_hitpoint(nullptr, 0.0f, 0.0f, RGL_AXIS_X), "node != nullptr");
}

TEST_F(GaussianNoiseAngularHitpointNodeTest, invalid_argument_st_dev)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_hitpoint(&gaussianNoiseNode, 0.0f, -1.0f, RGL_AXIS_X), "st_dev >= 0");
}

TEST_F(GaussianNoiseAngularHitpointNodeTest, invalid_argument_rotation_axis)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_hitpoint(&gaussianNoiseNode, 0.0f, 0.0f, (rgl_axis_t)4), "rotation_axis");
}

TEST_F(GaussianNoiseAngularHitpointNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&gaussianNoiseNode, 0.0f, 0.0f, RGL_AXIS_X));

    // If (*gaussianNoiseNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&gaussianNoiseNode, 2.0f, 2.0f, RGL_AXIS_X));
}
