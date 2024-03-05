#include "RGLFields.hpp"
#include "graph/Node.hpp"
#include <gtest/gtest.h>
#include "helpers/testPointCloud.hpp"
#include "helpers/sceneHelpers.hpp"

using namespace ::testing;

class VelocityDistortionTest : public RGLTest
{};

// TEST TODOs:
// - angular velocity
// - linear and angular velocities combined
// - distance field calculation

TEST_F(VelocityDistortionTest, smoke_test)
{
	rgl_node_t useRays = nullptr;
	rgl_node_t offsetsNode = nullptr;
	rgl_node_t raytrace = nullptr;
	rgl_node_t yield = nullptr;

	// Prepare scene
	spawnCubeOnScene(Mat3x4f::identity());

	// Generate 10 rays in the same origin looking at the same direction with time offset of firing.
	const int rayCount = 10;
	const float rayTimeOffsetToPrevious = 300.0f; // in milliseconds
	std::vector<rgl_mat3x4f> rays(rayCount);
	std::vector<float> timeOffsets(rayCount);
	for (int i = 0; i < rayCount; ++i) {
		rays[i] = Mat3x4f::identity().toRGL();
		timeOffsets[i] = i * rayTimeOffsetToPrevious;
	}

	const Vec3f expectedHitpoint = {0.0f, 0.0f, 1.0f};

	const rgl_vec3f linearVelocity = {0.0f, 1.0f, 0.0f};
	const rgl_vec3f angularVelocity = {0.0f, 0.0f, 0.0f};

	// Some rays will be fired out of cube
	auto isRayIndexOutOfCube = [&](int index) {
		const static float cubeSize = 2.0f;
		return linearVelocity.value[1] * rayTimeOffsetToPrevious * 0.001 * index > cubeSize / 2.0f;
	};

	std::vector<rgl_field_t> outFields{XYZ_VEC3_F32, IS_HIT_I32};

	// Create nodes
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&offsetsNode, timeOffsets.data(), rayCount));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_velocity(raytrace, &linearVelocity, &angularVelocity));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_distortion(raytrace, true));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yield, outFields.data(), outFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, offsetsNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(offsetsNode, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, yield));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	TestPointCloud outPointCloud = TestPointCloud::createFromNode(raytrace, outFields);
	auto outPoints = outPointCloud.getFieldValues<XYZ_VEC3_F32>();
	auto outIsHit = outPointCloud.getFieldValues<IS_HIT_I32>();

	for (int i = 0; i < rayCount; ++i) {
		if (isRayIndexOutOfCube(i)) {
			EXPECT_FALSE(outIsHit[i]);
			continue;
		}
		// Rays that hit the cube should have the same hit point (the same distance between sensor origin and cube)
		EXPECT_TRUE(outIsHit[i]);
		EXPECT_EQ(outPoints[i].x(), expectedHitpoint.x());
		EXPECT_EQ(outPoints[i].y(), expectedHitpoint.y());
		EXPECT_EQ(outPoints[i].z(), expectedHitpoint.z());
	}

	// Perform raytrace without velocity distortion -> all rays should hit the same point
	ASSERT_RGL_SUCCESS(rgl_node_raytrace_configure_distortion(raytrace, false));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	outPointCloud = TestPointCloud::createFromNode(raytrace, outFields);
	outPoints = outPointCloud.getFieldValues<XYZ_VEC3_F32>();
	outIsHit = outPointCloud.getFieldValues<IS_HIT_I32>();

	for (int i = 0; i < rayCount; ++i) {
		EXPECT_TRUE(outIsHit[i]);
		EXPECT_EQ(outPoints[i].x(), expectedHitpoint.x());
		EXPECT_EQ(outPoints[i].y(), expectedHitpoint.y());
		EXPECT_EQ(outPoints[i].z(), expectedHitpoint.z());
	}
}
