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
#include <complex>
#include "helpers/testPointCloud.hpp"

struct RcsAngleDistributionTest : RGLTest
{};

const auto minAzimuth = -65.0f;
const auto maxAzimuth = 65.0f;
const auto minElevation = -7.5f;
const auto maxElevation = 7.5f;
const auto azimuthStep = 0.05f;
const auto elevationStep = 0.05f;
const auto azimuthRad = (maxAzimuth - minAzimuth) * M_PIf / 180.0f;
const auto elevationRad = (maxElevation - minElevation) * M_PIf / 180.0f;

std::vector<rgl_mat3x4f> genRadarRays()
{
	std::vector<rgl_mat3x4f> rays;
	for (auto a = minAzimuth; a <= maxAzimuth; a += azimuthStep) {
		for (auto e = minElevation; e <= maxElevation; e += elevationStep) {
			// By default, the rays are directed along the Z-axis
			// So first, we rotate them around the Y-axis to point towards the X-axis (to be RVIZ2 compatible)
			// Then, rotation around Z is azimuth, around Y is elevation
			auto ray = Mat3x4f::rotationDeg(0, e + 90, -a);
			rays.emplace_back(ray.toRGL());

			// The above will have to be modified again - we assume that target is farther in X axis when in fact
			// we use Z as RGL LiDAR front. Remember to update.
		}
	}

	return rays;
}

TEST_F(RcsAngleDistributionTest, rotating_reflector_2d)
{
	GTEST_SKIP();
	// Load mesh
	rgl_mesh_t reflector2dMesh = loadFromSTL("../../test/data/reflector2d.stl");

	// Setup scene
	rgl_entity_t reflector2d = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2d, nullptr, reflector2dMesh));

	// Setup sensor and graph
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, DISTANCE_F32, NORMAL_VEC3_F32, RAY_IDX_U32, AZIMUTH_F32, ELEVATION_F32};
	std::vector<rgl_mat3x4f> rays = genRadarRays();
	rgl_node_t raysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, formatNode = nullptr, ros2Node = nullptr;
	rgl_node_t radarNode = nullptr, yieldNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactNode, RGL_FIELD_IS_HIT_I32));
	rgl_vec3f rangedSeparations{100.0f, 0.2f, 0.2f};
	EXPECT_RGL_SUCCESS(rgl_node_points_radar_postprocess(&radarNode, &rangedSeparations, 1, 0.2f, azimuthStep, elevationStep, 79E9f));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2Node, "rgl_test_topic", "rgl_test_frame_id"));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, radarNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(radarNode, formatNode));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2Node));

	for (float angle = -45; angle <= 45; angle += 0.1f) {
		auto position = Vec3f{5, 0, 0};
		auto rotation = Vec3f{0, 0, angle};
		auto scale = Vec3f{1, 1, 1};

		rgl_mat3x4f reflectorPose = Mat3x4f::TRS(position, rotation, scale).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2d, &reflectorPose));

		EXPECT_RGL_SUCCESS(rgl_graph_run(raysNode));
		int32_t pointCount = 0, pointSize = 0;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(radarNode, XYZ_VEC3_F32, &pointCount, &pointSize));
	}
}
