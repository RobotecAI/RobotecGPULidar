#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include "utils.hpp"
#include "scenes.hpp"

using namespace ::testing;

class VelocityDistortionTest : public RGLTest {};

// TEST TODOs:
// - angular velocity
// - linear and angular velocities combined
// - distance field calculation

TEST_F(VelocityDistortionTest, smoke_test)
{
	rgl_node_t useRays = nullptr;
	rgl_node_t offsetsNode = nullptr;
	rgl_node_t raytrace = nullptr;

	// Prepare scene
	spawnCubeOnScene(nullptr, Mat3x4f::identity());

	// Generate 10 rays in the same origin looking at the same direction with time offset of firing.
	const int rayCount = 10;
	const float rayTimeOffsetToPrevious = 300.0f; // in milliseconds
	std::vector<rgl_mat3x4f> rays(rayCount);
	std::vector<float> timeOffsets(rayCount);
	for (int i = 0; i < rayCount; ++i)
	{
		rays[i] = Mat3x4f::identity().toRGL();
		timeOffsets[i] = i * rayTimeOffsetToPrevious;
	}

	const Vec3f expectedHitpoint = {0.0f, 0.0f, 1.0f};

	const rgl_vec3f linearVelocity = { 0.0f, 1.0f, 0.0f };
	const rgl_vec3f angularVelocity = { 0.0f, 0.0f, 0.0f };

	// Some rays will be fired out of cube
	auto isRayIndexOutOfCube = [&](int index){
		const static float cubeSize = 2.0f;
		return linearVelocity.value[1] * rayTimeOffsetToPrevious * 0.001 * index > cubeSize / 2.0f;
	};

	// Create nodes
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&offsetsNode, timeOffsets.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_with_distortion(&raytrace, nullptr, &linearVelocity, &angularVelocity));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, offsetsNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(offsetsNode, raytrace));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	std::vector<::Field<XYZ_F32>::type> outPoints;
	outPoints.resize(rayCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, XYZ_F32, outPoints.data()));

	for (int i = 0; i < rayCount; ++i) {
		// Rays that hit the cube should have the same hit point (the same distance between sensor origin and cube)
		Vec3f toCompare = isRayIndexOutOfCube(i) ? Vec3f(NON_HIT_VALUE): expectedHitpoint;
		EXPECT_NEAR(outPoints[i].x(), toCompare.x(), EPSILON_F);
		EXPECT_NEAR(outPoints[i].y(), toCompare.y(), EPSILON_F);
		EXPECT_NEAR(outPoints[i].z(), toCompare.z(), EPSILON_F);
	}

	// Perform raytrace without velocity distortion -> all rays should hit the same point
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, XYZ_F32, outPoints.data()));

	for (int i = 0; i < rayCount; ++i) {
		EXPECT_NEAR(outPoints[i].x(), expectedHitpoint.x(), EPSILON_F);
		EXPECT_NEAR(outPoints[i].y(), expectedHitpoint.y(), EPSILON_F);
		EXPECT_NEAR(outPoints[i].z(), expectedHitpoint.z(), EPSILON_F);
	}
}
