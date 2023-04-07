#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class GaussianNoiseDistanceNodeTest : public RGLTest { };

TEST_F(GaussianNoiseDistanceNodeTest, invalid_arguments)
{
    rgl_node_t gaussianNoiseNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(nullptr, 0.0f, 0.0f, 0.0f), "node != nullptr");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.0f, -1.0f, 0.0f), "st_dev_base >= 0");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.0f, 0.0f, -1.0f), "st_dev_rise_per_meter >= 0");
}

TEST_F(GaussianNoiseDistanceNodeTest, valid_arguments)
{
    rgl_node_t gaussianNoiseNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));

    // If (*gaussianNoiseNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));
}
