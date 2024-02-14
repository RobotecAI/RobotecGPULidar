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

struct numpunct : std::numpunct<char> {
protected:
	char do_decimal_point() const override
	{
		return ',';
	}
};

struct RcsAngleDistributionTest : RGLTest
{};

const auto minAzimuth = -65.0f;
const auto maxAzimuth = 65.0f;
const auto minElevation = -7.5f;
const auto maxElevation = 7.5f;
const auto azimuthStep = 0.49f;
const auto elevationStep = 0.49f;
const auto azimuthRad = (maxAzimuth - minAzimuth) * M_PIf / 180.0f;
const auto elevationRad = (maxElevation - minElevation) * M_PIf / 180.0f;


Vec3f reflectPolarization(const Vec3f& pol, const Vec3f& hitNormal, const Vec3f& rayDir)
{
	const auto diffCrossNormal = rayDir.cross(hitNormal);
	const auto polU = diffCrossNormal.normalized();
	const auto polR = rayDir.cross(polU).normalized();

	const auto refDir = (rayDir - hitNormal * (2 * rayDir.dot(hitNormal))).normalized();
	const auto refPolU = -1.0f * polU;
	const auto refPolR = rayDir.cross(refPolU);

	const auto polCompU = pol.dot(polU);
	const auto polCompR = pol.dot(polR);

	return -polCompR * refPolR + polCompU * refPolU;
}

float getRayArea(float R) { return R * R * std::sin(elevationRad) * azimuthRad; }

float computeRCSOfCluster(int hitCount, const std::vector<uint32_t>& hitRayIdx, const std::vector<float>& hitDist,
                          const std::vector<Vec3f>& hitPos, const std::vector<Vec3f>& hitNorm, int rayCount,
                          const std::vector<rgl_mat3x4f> rayPose)
{
	constexpr float c0 = 299792458.0f;
	constexpr float freq = 79E9f;
	constexpr float waveLen = c0 / freq;
	constexpr float waveNum = 2.0f * M_PIf / waveLen;
	constexpr float reflectionCoef = 1.0f; // TODO
	constexpr std::complex<float> i = {0, 1.0};
	const Vec3f dirX = {1, 0, 0};
	const Vec3f dirY = {0, 1, 0};
	const Vec3f dirZ = {0, 0, 1};

	std::complex<float> AU = 0;
	std::complex<float> AR = 0;
	for (int hitIdx = 0; hitIdx < hitCount; ++hitIdx) {
		Vec3f rayDirCts = Mat3x4f::fromRGL(rayPose[hitRayIdx[hitIdx]]) * Vec3f{0, 0, 1};
		Vec3f rayDirSph = {rayDirCts.length(),
		                   rayDirCts[0] == 0 && rayDirCts[1] == 0 ? 0 : std::atan2(rayDirCts.y(), rayDirCts.x()),
		                   std::acos(rayDirCts.z() / rayDirCts.length())};
		float phi = rayDirSph[1]; // azimuth, 0 = X-axis, positive = CCW
		float the = rayDirSph[2]; // elevation, 0 = Z-axis, 90 = XY-plane, -180 = negative Z-axis

		// Consider unit vector of the ray direction, these are its projections:
		float cp = cosf(phi); // X-dir component
		float sp = sinf(phi); // Y-dir component
		float ct = cosf(the); // Z-dir component
		float st = sinf(the); // XY-plane component

		Vec3f dirP = {-sp, cp, 0};
		Vec3f dirT = {cp * ct, sp * ct, -st};

		std::complex<float> kr = {waveNum * hitDist[hitIdx], 0.0f};

		Vec3f rayDir = rayDirCts.normalized();
		Vec3f rayPol = Mat3x4f::fromRGL(rayPose[hitRayIdx[hitIdx]]) * Vec3f{-1, 0, 0}; // UP, perpendicular to ray
		Vec3f reflectedPol = reflectPolarization(rayPol, hitNorm[hitIdx], rayDir);

		Vector<3, std::complex<float>> rayPolCplx = {reflectedPol.x(), reflectedPol.y(), reflectedPol.z()};

		Vector<3, std::complex<float>> apE = reflectionCoef * exp(i * kr) * rayPolCplx;
		Vector<3, std::complex<float>> apH = -apE.cross(rayDir);

		Vec3f vecK = waveNum * ((dirX * cp + dirY * sp) * st + dirZ * ct);

		std::complex<float> BU = (-(apE.cross(-dirP) + apH.cross(dirT))).dot(rayDir);
		std::complex<float> BR = (-(apE.cross(dirT) + apH.cross(dirP))).dot(rayDir);
		std::complex<float> factor = std::complex<float>(0.0, ((waveNum * getRayArea(hitDist[hitIdx])) / (4.0f * M_PIf))) *
		                             exp(-i * vecK.dot(hitPos[hitIdx]));

		// TODO sum over rays
		AU += BU * factor;
		AR += BR * factor;
	}
	float rcs = 10.0f * log10f(4.0f * M_PIf * (pow(abs(AU), 2) + pow(abs(AR), 2)));
	return rcs;
}

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
	// Load mesh
	rgl_mesh_t reflector2dMesh = loadFromSTL("../../test/data/reflector2d.stl");

	// Setup scene
	rgl_entity_t reflector2d = nullptr;
	EXPECT_RGL_SUCCESS(rgl_entity_create(&reflector2d, nullptr, reflector2dMesh));

	// Box to visualize all rays
	//	rgl_entity_t entity = makeEntity(makeCubeMesh());
	//	auto epose = Mat3x4f::scale(10, 10, 10).toRGL();
	//	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &epose));

	// Setup sensor and graph
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, DISTANCE_F32, NORMAL_VEC3_F32, RAY_IDX_U32};
	std::vector<rgl_mat3x4f> rays = genRadarRays();
	rgl_node_t raysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, formatNode = nullptr, ros2Node = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactNode, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2Node, "rgl_test_topic", "rgl_test_frame_id"));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2Node));

	std::locale loc(std::locale(), new numpunct());
	fmt::print("pointCount,angle,rcs\n");
	for (float angle = -45; angle <= 45; angle += 0.1f) {
		//		Vec3f scale = {0.2, 0.2, 0.2};
		Vec3f scale = {1, 1, 1};

		rgl_mat3x4f reflectorPose = Mat3x4f::TRS({5, 0, 0}, {0, 0, angle}, scale).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(reflector2d, &reflectorPose));

		EXPECT_RGL_SUCCESS(rgl_graph_run(raysNode));
		TestPointCloud pointCloud = TestPointCloud::createFromNode(compactNode, fields);
		auto rcs = computeRCSOfCluster(pointCloud.getPointCount(), pointCloud.getFieldValues<RAY_IDX_U32>(),
		                               pointCloud.getFieldValues<DISTANCE_F32>(), pointCloud.getFieldValues<XYZ_VEC3_F32>(),
		                               pointCloud.getFieldValues<NORMAL_VEC3_F32>(), rays.size(), rays);
		std::cout << fmt::format(loc, "{0};{1:L};{2:L}", pointCloud.getPointCount(), angle, rcs) << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
