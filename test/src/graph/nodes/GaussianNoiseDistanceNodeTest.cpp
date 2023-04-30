#include <utils.hpp>

class GaussianNoiseDistanceNodeTest : public RGLTest {
protected:
    rgl_node_t gaussianNoiseNode;

    GaussianNoiseDistanceNodeTest()
    {
        gaussianNoiseNode = nullptr;
    }
};

TEST_F(GaussianNoiseDistanceNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(nullptr, 0.0f, 0.0f, 0.0f), "node != nullptr");
}

TEST_F(GaussianNoiseDistanceNodeTest, invalid_argument_st_dev_base)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.0f, -1.0f, 0.0f), "st_dev_base >= 0");
}

TEST_F(GaussianNoiseDistanceNodeTest, invalid_argument_st_dev_rise_per_meter)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.0f, 0.0f, -1.0f), "st_dev_rise_per_meter >= 0");
}

TEST_F(GaussianNoiseDistanceNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));

    // If (*gaussianNoiseNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));
}
