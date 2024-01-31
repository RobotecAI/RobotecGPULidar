#include <stdexcept>
#include <thread>
#include <chrono>
#include <gtest/gtest.h>

#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <spdlog/fmt/bundled/format.h>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>

struct RcsAngleDistributionTest : RGLTest
{};

TEST_F(RcsAngleDistributionTest, rotating_reflector_2d)
{
	GTEST_SKIP();
	// Load mesh
	rgl_mesh_t reflector2dMesh = loadFromSTL("data/reflector2d.stl");

	// Setup scene
	rgl_entity_t reflector2d = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2d, nullptr, reflector2dMesh));
	rgl_mat3x4f reflectorPose = Mat3x4f::TRS({5, 0, 0}).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2d, &reflectorPose));

	// Setup sensor and graph
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32};
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180);
	rgl_node_t raysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, formatNode = nullptr, ros2Node = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2Node, "rgl_test_topic", "rgl_test_frame_id"));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2Node));

	float t = 0;
	while (true) {
		EXPECT_RGL_SUCCESS(rgl_graph_run(raysNode));
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		rgl_mat3x4f reflectorPose = Mat3x4f::TRS({5, 0, 0}, {0, 0, -45 + 90 * t}).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2d, &reflectorPose));
		t += 0.01;
		t = t - std::floor(t);
		// TODO: Implement clustering and plot RCS vs angle
	}
}