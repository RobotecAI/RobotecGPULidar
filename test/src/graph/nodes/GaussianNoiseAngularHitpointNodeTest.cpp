#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class GaussianNoiseAngularHitpointNodeTest : public RGLAutoCleanupTest { };

TEST_F(GaussianNoiseAngularHitpointNodeTest, invalid_arguments)
{
    // TODO(nebraszka): Parameterize the test to take a permutation of the set of all axes.
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_hitpoint(nullptr, 0.0f, 0.0f, RGL_AXIS_X), "node != nullptr");
}

TEST_F(GaussianNoiseAngularHitpointNodeTest, valid_arguments)
{
    rgl_node_t gaussianNoiseNode = nullptr;
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&gaussianNoiseNode, 0.0f, 0.0f, RGL_AXIS_X));

	// If (*gaussianNoiseNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_hitpoint(&gaussianNoiseNode, 2.0f, 2.0f, RGL_AXIS_X));
}
