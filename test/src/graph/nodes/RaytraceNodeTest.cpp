#include <helpers/commonHelpers.hpp>

class RaytraceNodeTest : public RGLTest
{
protected:
	rgl_node_t raytraceNode;

	RaytraceNodeTest() { raytraceNode = nullptr; }
};

TEST_F(RaytraceNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr), "node != nullptr");
}

TEST_F(RaytraceNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	ASSERT_THAT(raytraceNode, testing::NotNull());

	// If (*raytraceNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
}

// Configuration tests
TEST_F(RaytraceNodeTest, config_velocity_invalid_argument_node)
{
	rgl_node_t raytraceNode = nullptr;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_velocity(raytraceNode, nullptr, nullptr), "node != nullptr");
}

TEST_F(RaytraceNodeTest, config_velocity_invalid_argument_linear_velocity)
{
	rgl_node_t raytraceNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_velocity(raytraceNode, nullptr, nullptr),
	                            "linear_velocity != nullptr");
}

TEST_F(RaytraceNodeTest, config_velocity_invalid_argument_angular_velocity)
{
	rgl_node_t raytraceNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	const rgl_vec3f linearVelocity = {1.0f, 2.0f, 3.0f};
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_velocity(raytraceNode, &linearVelocity, nullptr),
	                            "angular_velocity != nullptr");
}

TEST_F(RaytraceNodeTest, config_velocity_invalid_node_object)
{
	const rgl_vec3f linearVelocity = {1.0f, 2.0f, 3.0f};
	const rgl_vec3f angularVelocity = {4.0f, 5.0f, 6.0f};
	EXPECT_RGL_INVALID_OBJECT(rgl_node_raytrace_configure_velocity((rgl_node_t) 0x1234, &linearVelocity, &angularVelocity),
	                          "Object does not exist: Node 0x1234");
}

TEST_F(RaytraceNodeTest, config_velocity_valid_arguments)
{
	rgl_node_t raytraceNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	const rgl_vec3f linearVelocity = {1.0f, 2.0f, 3.0f};
	const rgl_vec3f angularVelocity = {4.0f, 5.0f, 6.0f};
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_velocity(raytraceNode, &linearVelocity, &angularVelocity));
}

TEST_F(RaytraceNodeTest, config_distortion_invalid_argument_node)
{
	rgl_node_t raytraceNode = nullptr;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_distortion(raytraceNode, false), "node != nullptr");
}

TEST_F(RaytraceNodeTest, config_distortion_invalid_node_object)
{
	EXPECT_RGL_INVALID_OBJECT(rgl_node_raytrace_configure_distortion((rgl_node_t) 0x1234, false),
	                          "Object does not exist: Node 0x1234");
}

TEST_F(RaytraceNodeTest, config_distortion_valid_arguments)
{
	rgl_node_t raytraceNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_distortion(raytraceNode, true));
}