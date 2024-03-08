#include <stdexcept>
#include <thread>
#include <chrono>
#include <filesystem>
#include <complex>
#include <gtest/gtest.h>

#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <spdlog/fmt/bundled/format.h>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>
#include "helpers/testPointCloud.hpp"

struct RadarTest : RGLTest
{};

const auto minAzimuth = -65.0f;
const auto maxAzimuth = 65.0f;
const auto minElevation = -7.5f;
const auto maxElevation = 7.5f;
const auto azimuthStep = 0.49f;
const auto elevationStep = 0.49f;

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

namespace fs = std::filesystem;
using namespace std::chrono_literals;

TEST_F(RadarTest, rotating_reflector_2d)
{
	//	GTEST_SKIP();

	// Setup sensor and graph
	std::vector<rgl_mat3x4f> raysData = genRadarRays();
	rgl_radar_scope_t radarScope{
	    .begin_distance = 1.3f,
	    .end_distance = 19.0f,
	    .distance_separation_threshold = 0.3f,
	    .radial_speed_separation_threshold = 0.3f,
	    .azimuth_separation_threshold = 8.0f * (std::numbers::pi_v<float> / 180.0f),
	};
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, DISTANCE_F32, RCS_F32, POWER_F32, NOISE_F32, SNR_F32};

	// LiDAR
	rgl_node_t rays = nullptr, noise = nullptr, raytrace = nullptr, compact = nullptr, lidarFormat = nullptr,
	           lidarPublish = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysData.data(), raysData.size()));
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&noise, 0, 2, RGL_AXIS_Z));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&lidarFormat, fields.data(), 1)); // Publish only XYZ
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&lidarPublish, "rgl_lidar", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, noise));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(noise, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, lidarFormat));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarFormat, lidarPublish));

	// Radar postprocessing and publishing
	rgl_node_t radarPostProcess = nullptr, radarFormat = nullptr, radarPublish = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_radar_postprocess(&radarPostProcess, &radarScope, 1, azimuthStep, elevationStep, 79E9f));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&radarFormat, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&radarPublish, "rgl_radar", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, radarPostProcess));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(radarPostProcess, radarFormat));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(radarFormat, radarPublish));

	// Setup scene
	std::vector<rgl_entity_t> reflectors2D;
	reflectors2D.resize(3, nullptr);
	rgl_mesh_t reflector2dMesh = loadFromSTL(fs::path(RGL_TEST_DATA_DIR) / "reflector2d.stl");
	for (auto&& reflector2D : reflectors2D) {
		EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2D, nullptr, reflector2dMesh));
	}

	uint64_t time = 0;
	for (float angle = -45; angle <= 45; angle += 0.1f) {
		EXPECT_RGL_SUCCESS(rgl_scene_set_time(nullptr, time));
		auto position = Vec3f{5, 0, 0};
		auto rotation = Vec3f{0, 0, angle};
		auto scale = Vec3f{1, 1, 1};

		auto offset = Vec3f{0, -3, 0};
		for (auto&& reflector2D : reflectors2D) {
			rgl_mat3x4f reflectorPose = Mat3x4f::TRS(position + offset, rotation, scale).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2D, &reflectorPose));
			offset += Vec3f{0, 3, 0};
		}

		fmt::print("Angle: {}\n", angle);
		EXPECT_RGL_SUCCESS(rgl_graph_run(rays));
		std::this_thread::sleep_for(100ms);
		time += 10'000'000; // 10 ms
	}
}
