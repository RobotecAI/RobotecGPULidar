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

float computeRCSOfCluster(int rayCount, const std::vector<rgl_mat3x4f> rayPose, const std::vector<float>& distance,
                          const std::vector<Vec3f>& hitPos)
{
	constexpr float c0 = 299792458.0f;
	constexpr float freq = 79E9;
	constexpr float angFreq = 2.0f * M_PIf * freq;
	constexpr float waveLen = c0 / freq;
	constexpr float waveNum = 2.0f * M_PIf / waveLen;
	constexpr float reflectionCoef = 1.0f; // TODO
	constexpr float rayArea = 1.0f;        // TODO
	constexpr std::complex<float> i = {0, 1.0};
	const Vec3f dirX = {1, 0, 0};
	const Vec3f dirY = {0, 1, 0};
	const Vec3f dirZ = {0, 0, 1};

	std::complex<float> AU = 0;
	std::complex<float> AR = 0;
	for (int rayIdx = 0; rayIdx < rayCount; ++rayIdx) {
		Vec3f rayDirCts = Mat3x4f::fromRGL(rayPose[rayIdx]) * Vec3f{0, 0, 1};
		Vec3f rayDirSph = {rayDirCts.length(),
		                   rayDirCts[0] == 0 && rayDirCts[1] == 0 ? 0 : std::atan2(rayDirCts.y(), rayDirCts.x()),
		                   std::acos(rayDirCts.z() / rayDirCts.length())};
		float phi = rayDirSph[1]; // azimuth, 0 = X-axis, positive = CCW
		float the = rayDirSph[2]; // elevation, 0 = Z-axis, 90 = XY-plane, -180 = negative Z-axis

		// TODO: validate it, it doesnt make sense when visualized
		// Consider unit vector of the ray direction, these are its projections:
		float cp = cosf(phi); // X-dir component
		float sp = sinf(phi); // Y-dir component
		float ct = cosf(the); // Z-dir component
		float st = sinf(the); // XY-plane component

		Vec3f dirP = {-sp, cp,
		              0}; // Vector perpendicular to ray's projection onto XY plane // TODO: ensure, validation does not confirm
		Vec3f dirT = {cp * ct, sp * st, -st}; // TODO: What is this?

		float kr = waveNum * distance[rayIdx];

		Vec3f rayDir = rayDirCts.normalized(); // TODO: Is this OK?
		Vec3f rayPol = {0, 0, 1};              // TODO!

		Vector<3, std::complex<float>> apE = exp(i * kr) * reflectionCoef;
		Vector<3, std::complex<float>> apH = -apE.cross(rayDir);

		Vec3f vecK = waveNum * ((dirX * cp + dirY * sp) * st + dirZ * ct);

		// TODO: dot product may be not commutative for complex numbers
		std::complex<float> BU = static_cast<Vector<3, std::complex<float>>>(rayDir).dot(-(apE.cross(-dirP) + apH.cross(dirT)));
		std::complex<float> BR = static_cast<Vector<3, std::complex<float>>>(rayDir).dot(-(apE.cross(dirT) + apH.cross(dirP)));
		std::complex<float> factor = std::complex<float>(0.0f, ((waveNum * rayArea) / (4.0f * M_PIf))) *
		                             exp(-i * hitPos[rayIdx].dot(vecK));

		// TODO sum over rays
		AU += BU * factor;
		AR += BR * factor;
	}
	float rcs = 4.0f * M_PIf * (pow(abs(AU), 2) + pow(abs(AR), 2));
	return rcs;
}

struct RcsAngleDistributionTest : RGLTest
{};

TEST_F(RcsAngleDistributionTest, rotating_reflector_2d)
{
	// Load mesh
	rgl_mesh_t reflector2dMesh = loadFromSTL("../../test/data/reflector2d.stl");

	// Setup scene
	rgl_entity_t reflector2d = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2d, nullptr, reflector2dMesh));
	rgl_mat3x4f reflectorPose = Mat3x4f::TRS({5, 0, 0}).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2d, &reflectorPose));

	// Setup sensor and graph
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, DISTANCE_F32};
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


	fmt::print("pointCount,angle,rcs\n");
	for (float angle = -45; angle <= 45; angle += 0.1f) {
		EXPECT_RGL_SUCCESS(rgl_graph_run(raysNode));
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		rgl_mat3x4f reflectorPose = Mat3x4f::TRS({5, 0, 0}, {0, 0, angle}).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2d, &reflectorPose));

		TestPointCloud pointCloud = TestPointCloud::createFromNode(compactNode, {XYZ_VEC3_F32, DISTANCE_F32});
		auto rcs = computeRCSOfCluster(pointCloud.getPointCount(), rays, pointCloud.getFieldValues<DISTANCE_F32>(),
		                               pointCloud.getFieldValues<XYZ_VEC3_F32>());
		fmt::print("{},{},{}\n", pointCloud.getPointCount(), angle, rcs);
	}
}