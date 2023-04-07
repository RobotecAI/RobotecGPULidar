#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class GaussianNoiseAngularRayNodeTest : public RGLTest { };

TEST_F(GaussianNoiseAngularRayNodeTest, invalid_arguments)
{
    rgl_node_t gaussianNoiseNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(nullptr, 0.0f, 0.0f, RGL_AXIS_X), "node != nullptr");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, -1.0f, RGL_AXIS_X), "st_dev >= 0");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, 0.0f, (rgl_axis_t)4), "rotation_axis");
}

TEST_F(GaussianNoiseAngularRayNodeTest, valid_arguments)
{
    // TODO(nebraszka): Parameterize the test to take a permutation of the set of all axes.
    rgl_node_t gaussianNoiseNode = nullptr;
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, 0.0f, RGL_AXIS_X));

	// If (*gaussianNoiseNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 2.0f, 2.0f, RGL_AXIS_X));
}
