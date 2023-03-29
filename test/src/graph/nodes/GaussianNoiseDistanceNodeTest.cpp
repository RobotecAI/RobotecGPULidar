#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class GaussianNoiseDistanceNodeTest : public RGLTest { };

TEST_F(GaussianNoiseDistanceNodeTest, invalid_arguments)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(nullptr, 0.0f, 0.0f, 0.0f), "node != nullptr");
}

TEST_F(GaussianNoiseDistanceNodeTest, valid_arguments)
{
    rgl_node_t gaussianNoiseNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));

    // If (*gaussianNoiseNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));
}
