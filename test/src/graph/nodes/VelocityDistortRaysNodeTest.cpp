#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include "utils.hpp"
#include "scenes.hpp"

using namespace ::testing;

class VelocityDistortRaysNodeTest : public RGLTest, public RGLPointTestHelper {};

// TEST TODOS:
// -angular velocity
// -linear and angular velocities combined
// -pipeline with transform node (before / after distortion)
// -distance field calculation

TEST_F(VelocityDistortRaysNodeTest, invalid_arguments)
{
	rgl_node_t velocityDistortRaysNode;
	rgl_vec3f velocity;
	rgl_vec3f angularVelocity;

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(nullptr, nullptr, nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(nullptr, &velocity, &angularVelocity), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, nullptr, &angularVelocity), "linear_velocity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &velocity, nullptr), "angular_velocity != nullptr");
}

TEST_F(VelocityDistortRaysNodeTest, valid_arguments)
{
	rgl_node_t velocityDistortRaysNode = nullptr;
	rgl_vec3f velocity = { 1.0f, 1.0f, 1.0f };
	rgl_vec3f angularVelocity = { 0.0f, 0.0f, 1.0f };

	EXPECT_RGL_SUCCESS(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &velocity, &angularVelocity));
	ASSERT_THAT(velocityDistortRaysNode, testing::NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &velocity, &angularVelocity));
}

TEST_F(VelocityDistortRaysNodeTest, use_case)
{
	rgl_node_t useRays = nullptr;
	rgl_node_t transformRays = nullptr;
	rgl_node_t offsetsNode = nullptr;
	rgl_node_t velocityDistortRaysNode = nullptr;
	rgl_node_t raytrace = nullptr;

	// Prepare scene
	setupBoxesAlongAxes(nullptr);

	// Generate 10 rays in the same origin looking at the same x direction
	// After that generate 10 offsets for the rays. each ray will have slightly bigger offset in y axis
	const float rayCount = 10;
	const float rayTimeOffset = 0.2f; // One millisecond
	std::vector<rgl_mat3x4f> rays(rayCount);
	std::vector<float> timeOffsets(rayCount);
	for (int i = 0; i < rayCount; ++i)
	{
            rays[i] = Mat3x4f::translation(1, 0, 0).toRGL();
            timeOffsets[i] = i * rayTimeOffset;
	}

	const rgl_vec3f lidarVelocity = { 0.0f, 0.8f, 0.0f };
	const rgl_vec3f lidarangularVelocity = { 0.0f, 0.0f, 0.0f };

	// Create nodes
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&offsetsNode, timeOffsets.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_rays_velocity_distort(&velocityDistortRaysNode, &lidarVelocity, &lidarangularVelocity));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));

	// Connect nodes into graph without velocity distortion.
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, offsetsNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(offsetsNode, raytrace));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	std::vector<::Field<XYZ_F32>::type> outPoints;
	outPoints.resize(rayCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, XYZ_F32, outPoints.data()));

	//Without velocity distortion all rays should hit the same point
	std::for_each(outPoints.begin(), outPoints.end(), [&](auto& point) {
		EXPECT_NEAR(point.x(), outPoints.begin()->x(), EPSILON_F);
		EXPECT_NEAR(point.y(), outPoints.begin()->y(), EPSILON_F);
		EXPECT_NEAR(point.z(), outPoints.begin()->z(), EPSILON_F);
	});

	// Rearrange graph in order to include velocity distortion node
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(offsetsNode, raytrace));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(offsetsNode, velocityDistortRaysNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(velocityDistortRaysNode, raytrace));

	// Run graph again with velocity distortion
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, XYZ_F32, outPoints.data()));

	// After distortion rays should hit with 0.1 offset in y axis, without changes in x and z axis.(
	for (int i = 0; i < rayCount; ++i)
	{
		EXPECT_NEAR(outPoints[i].x(), outPoints.begin()->x(), EPSILON_F);
		EXPECT_NEAR(outPoints[i].y(), outPoints.begin()->y() + ( i * rayTimeOffset * (lidarVelocity.value[1]* 0.001f)), EPSILON_F);
		EXPECT_NEAR(outPoints[i].z(), outPoints.begin()->z(), EPSILON_F);
	}
}
