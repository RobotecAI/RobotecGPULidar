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

Mat3x4f calculateMat(float azi, float ele, float rol)
{
	constexpr auto deg2rad = M_PIf / 180.0f;
	azi *= deg2rad;
	ele *= deg2rad;
	rol *= deg2rad;

	const auto cr = cosf(rol);
	const auto sr = sinf(rol);
	const auto ca = cosf(azi);
	const auto sa = sinf(azi);
	const auto ce = cosf(ele);
	const auto se = sinf(ele);

	Mat3x4f m = Mat3x4f::identity();

	m.rc[0][0] = -sa * se * sr + ca * cr;
	m.rc[0][1] = -sa * ce;
	m.rc[0][2] = sa * se * cr + sr * ca;
	m.rc[1][0] = sa * cr + se * sr * ca;
	m.rc[1][1] = ca * ce;
	m.rc[1][2] = sa * sr - se * ca * cr;
	m.rc[2][0] = -sr * ce;
	m.rc[2][1] = se;
	m.rc[2][2] = ce * cr;

//	m.rc[0][0] = cr * ca - sr * ce * sa;
//	m.rc[0][1] = sr * ca + cr * ce * sa;
//	m.rc[0][2] = se * sa;
//	m.rc[1][0] = -cr * sa - sr * ce * ca;
//	m.rc[1][1] = -sr * sa + cr * ce * ca;
//	m.rc[1][2] = se * ca;
//	m.rc[2][0] = sr * se;
//	m.rc[2][1] = -cr * se;
//	m.rc[2][2] = ce;

//	m.rc[0][0] = ca;
//	m.rc[0][1] = -sa;
//	m.rc[0][2] = 0;
//	m.rc[1][0] = sa;
//	m.rc[1][1] = ca;
//	m.rc[1][2] = 0;
//	m.rc[2][0] = 0;
//	m.rc[2][1] = 0;
//	m.rc[2][2] = 1;

	return m;
}

rgl_mesh_t getFlatPlate(float dim)
{
	dim *= 0.5f;
	std::vector<rgl_vec3f> rgl_vertices = {
	    { 0.0f, dim, -dim },
	    { 0.0f, dim, dim },
	    { 0.0f, -dim, dim },
	    { 0.0f, -dim, -dim }};
	std::vector<rgl_vec3i> rgl_indices = {
	    { 0, 3, 1 },
	    { 1, 3, 2 }
	};

	rgl_mesh_t outMesh = nullptr;
	rgl_status_t status = rgl_mesh_create(&outMesh, rgl_vertices.data(), rgl_vertices.size(), rgl_indices.data(),
	                                      rgl_indices.size());

	if (status != RGL_SUCCESS) {
		const char* errorString = nullptr;
		rgl_get_last_error_string(&errorString);
		throw std::runtime_error(fmt::format("rgl_mesh_create: {}", errorString));
	}
	return outMesh;
}

std::vector<rgl_mat3x4f> genRadarRays()
{
	std::vector<rgl_mat3x4f> rays;
	for (auto a = minAzimuth; a <= maxAzimuth; a += azimuthStep) {
		for (auto e = minElevation; e <= maxElevation; e += elevationStep) {
			// By default, the rays are directed along the Z-axis
			// So first, we rotate them around the Y-axis to point towards the X-axis (to be RVIZ2 compatible)
			// Then, rotation around Z is azimuth, around Y is elevation
			const auto ray = Mat3x4f::rotationDeg(a, e, 0);
			rays.emplace_back(ray.toRGL());

			// The above will have to be modified again - we assume that target is farther in X axis when in fact
			// we use Z as RGL LiDAR front. Remember to update.

			const auto rayDir = ray * Vec3f {0, 0, 1};
			//printf("rayDir: %.2f %.2f %.2f\n", rayDir.x(), rayDir.y(), rayDir.z());
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
//	std::vector<rgl_mat3x4f> raysData = {
//	    Mat3x4f::rotationDeg(0, 0, 0).toRGL(),
//	};
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

	rgl_node_t raysTransform = nullptr;
	auto raycasterTransform = Mat3x4f::rotationDeg(0.0f, 90.0f, 0.0f).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysData.data(), raysData.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&raysTransform, &raycasterTransform));
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&noise, 0, 2, RGL_AXIS_Z));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&lidarFormat, fields.data(), 1)); // Publish only XYZ
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&lidarPublish, "rgl_lidar", "world"));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, raytrace));
	//EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raysTransform, raytrace));
	//EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(noise, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, lidarFormat));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarFormat, lidarPublish));

	const float azimuthStepRad = azimuthStep * (std::numbers::pi_v<float> / 180.0f);
	const float elevationStepRad = elevationStep * (std::numbers::pi_v<float> / 180.0f);
	const float powerTransmittedDdm = 31.0f;
	const float antennaGainDbi = 27.0f;

	// Radar postprocessing and publishing
	rgl_node_t radarPostProcess = nullptr, radarFormat = nullptr, radarPublish = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_radar_postprocess(&radarPostProcess, &radarScope, 1, azimuthStepRad, elevationStepRad, 79E9f,
	                                                     powerTransmittedDdm, antennaGainDbi, 60.0f, 1.0f));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&radarFormat, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&radarPublish, "rgl_radar", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, radarPostProcess));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(radarPostProcess, radarFormat));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(radarFormat, radarPublish));

	rgl_entity_t reflector2D = nullptr;
	//rgl_mesh_t reflector2dMesh = loadFromSTL(fs::path(RGL_TEST_DATA_DIR) / "reflector2d.stl");
	rgl_mesh_t reflector2dMesh = getFlatPlate(1.0f);
	//rgl_mesh_t reflector2dMesh = loadFromSTL("/home/pawel/Pawel/Documentation/RGL/2024Q1/Lexus.stl");
	EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2D, nullptr, reflector2dMesh));

	float angle = -50.0f;
	for (; angle <= 50; angle += 0.1f) {
		//auto position = Vec3f{15 + angle / 5, 0, 0};
		auto position = Vec3f{0, 0, 5};
		auto rotation = Vec3f{0, -90, 0};
		auto scale = Vec3f{1, 1, 1};
		rgl_mat3x4f reflectorPose = (Mat3x4f::TRS(position, {angle, 0, 0}, scale) * Mat3x4f::TRS({0, 0, 0}, rotation, scale)).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2D, &reflectorPose));

		//printf("Angle: %.2f\n", angle);

		EXPECT_RGL_SUCCESS(rgl_graph_run(rays));
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

//		for (float azi = -15.0f; azi <= 15.0f; azi += 15.0f) {
//			//for (float ele = -15.0f; ele <= 15.0f; ele += 15.0f) {
//				raycasterTransform = Mat3x4f::rotationDeg(0.0f, 90.0f, -azi).toRGL();
//				EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&raysTransform, &raycasterTransform));
//				EXPECT_RGL_SUCCESS(rgl_graph_run(rays));
//			//}
//		}
	}

	// Setup scene
//	std::vector<rgl_entity_t> reflectors2D;
//	reflectors2D.resize(3, nullptr);
//	rgl_mesh_t reflector2dMesh = loadFromSTL(fs::path(RGL_TEST_DATA_DIR) / "reflector2d.stl");
//	for (auto&& reflector2D : reflectors2D) {
//		EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2D, nullptr, reflector2dMesh));
//	}
//
//	uint64_t time = 0;
//	for (float angle = -45; angle <= 45; angle += 0.1f) {
//		EXPECT_RGL_SUCCESS(rgl_scene_set_time(nullptr, time));
//		auto position = Vec3f{5, 0, 0};
//		auto rotation = Vec3f{0, 0, angle};
//		auto scale = Vec3f{1, 1, 1};
//
//		auto offset = Vec3f{0, -3, 0};
//		for (auto&& reflector2D : reflectors2D) {
//			rgl_mat3x4f reflectorPose = Mat3x4f::TRS(position + offset, rotation, scale).toRGL();
//			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2D, &reflectorPose));
//			offset += Vec3f{0, 3, 0};
//		}
//
//		EXPECT_RGL_SUCCESS(rgl_graph_run(rays));
//		time += 10'000'000; // 10 ms
//	}
}
