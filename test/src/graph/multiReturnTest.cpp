#include <helpers/commonHelpers.hpp>
#include "helpers/lidarHelpers.hpp"
#include "helpers/testPointCloud.hpp"
#include "helpers/sceneHelpers.hpp"

#include "RGLFields.hpp"
#include "math/Mat3x4f.hpp"
#include "gpu/MultiReturn.hpp"

#include <cmath>

using namespace std::chrono_literals;

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

class GraphMultiReturn : public RGLTest
{
protected:
	// VLP16 data
	const float vlp16LidarObjectDistance = 100.0f;
	const float vlp16LidarHBeamDivergence = 0.003f;  // Velodyne VLP16 horizontal beam divergence in rads
	const float vlp16LidarVBeamDivergence = 0.0015f; // Velodyne VLP16 vertical beam divergence in rads
	const float vlp16LidarHBeamDiameter = 0.2868f;   // Velodyne VLP16 beam horizontal diameter at 100m
	const float vlp16LidarVBeamDiameter = 0.1596f;   // Velodyne VLP16 beam vertical diameter at 100m

	std::vector<Mat3x4f> vlp16Channels{Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(11.2f)}, {0.0f, -15.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(0.7f)}, {0.0f, +1.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(9.7f)}, {0.0f, -13.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-2.2f)}, {0.0f, +3.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(8.1f)}, {0.0f, -11.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-3.7f)}, {0.0f, +5.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(6.6f)}, {0.0f, -9.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-5.1f)}, {0.0f, +7.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(5.1f)}, {0.0f, -7.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-6.6f)}, {0.0f, +9.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(3.7f)}, {0.0f, -5.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-8.1f)}, {0.0f, +11.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(2.2f)}, {0.0f, -3.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-9.7f)}, {0.0f, +13.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(0.7f)}, {0.0f, -1.0f, 0.0f}),
	                                   Mat3x4f::TRS({0.0f, 0.0f, mmToMeters(-11.2f)}, {0.0f, +15.0f, 0.0f})};

	const std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, IS_HIT_I32, DISTANCE_F32,
	                                         RETURN_TYPE_U8 /*, INTENSITY_F32, ENTITY_ID_I32*/};

	rgl_node_t rays = nullptr, cameraRays = nullptr, transform = nullptr, cameraTransform = nullptr, raytrace = nullptr,
	           cameraRaytrace = nullptr, format = nullptr, cameraFormat = nullptr, publish = nullptr, cameraPublish = nullptr,
	           compact = nullptr;

	void constructMRGraph(const std::vector<rgl_mat3x4f>& raysTf, const rgl_mat3x4f& lidarPose, const float hBeamDivAngle,
	                      const float vBeamDivAngle, rgl_return_mode_t returnMode, bool withPublish = false)
	{
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transform, &lidarPose));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_beam_divergence(raytrace, hBeamDivAngle, vBeamDivAngle));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_return_mode(raytrace, returnMode));
		EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
		EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
#if RGL_BUILD_ROS2_EXTENSION
		if (withPublish) {
			EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&publish, "MRTest_MultiReturnCloud", "world"));
		}
#else
		if (withPublish) {
			GTEST_SKIP() << "Publishing is not supported without ROS2 extension. Skippping the test.";
		}
#endif
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, transform));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transform, raytrace));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format));
		if (withPublish) {
			EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, publish));
		}
	}

	void constructCameraGraph(const rgl_mat3x4f& cameraPose)
	{
#if RGL_BUILD_ROS2_EXTENSION
		const std::vector<rgl_mat3x4f> cameraRayTf = makeLidar3dRays(360.0f, 180.0f, 1.0f, 1.0f);
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&cameraRays, cameraRayTf.data(), cameraRayTf.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&cameraTransform, &cameraPose));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&cameraRaytrace, nullptr));
		EXPECT_RGL_SUCCESS(rgl_node_points_format(&cameraFormat, fields.data(), fields.size()));
		EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&cameraPublish, "MRTest_Camera", "world"));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraRays, cameraTransform));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraTransform, cameraRaytrace));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraRaytrace, cameraFormat));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraFormat, cameraPublish));
#else
		GTEST_SKIP() << "Publishing is not supported without ROS2 extension. Skippping the test with camera.";
#endif
	}

	void spawnStairsOnScene(const float stepWidth, const float stepHeight, const float stepDepth, const float stairsBaseHeight,
	                        const Vec3f& stairsTranslation) const
	{
		const Vec3f cubeHalfEdgeScaled{CUBE_HALF_EDGE * stepDepth / CUBE_EDGE, CUBE_HALF_EDGE * stepWidth / CUBE_EDGE,
		                               CUBE_HALF_EDGE * stepHeight / CUBE_EDGE};

		const auto firstCubeTf =
		    Mat3x4f::translation(stairsTranslation) *
		    Mat3x4f::TRS({cubeHalfEdgeScaled.x(), 0.0f, -cubeHalfEdgeScaled.z() + stairsBaseHeight + stepHeight},
		                 {0.0f, 0.0f, 0.0f}, {stepDepth / CUBE_EDGE, stepWidth / CUBE_EDGE, stepHeight / CUBE_EDGE});
		spawnCubeOnScene(firstCubeTf);
		spawnCubeOnScene(Mat3x4f::translation(stepDepth, 0.0f, stepHeight) * firstCubeTf);
		spawnCubeOnScene(Mat3x4f::translation(2 * stepDepth, 0.0f, 2 * stepHeight) * firstCubeTf);
	}

	constexpr float mmToMeters(float mm) const { return mm * 0.001f; }
};

/**
 * This test verifies the accuracy of multiple return handling for the data specified for LiDAR VLP16
 * by firing a single beam into a cube and making sure the first and last hits are correctly calculated.
 */
TEST_F(GraphMultiReturn, vlp16_data_compare)
{
	GTEST_SKIP();

	// Lidar
	const std::vector<rgl_mat3x4f> raysTf{Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, -90.0f}).toRGL()};
	const float lidarCubeFaceDist = vlp16LidarObjectDistance;
	const float lidarCubeCenterDist = lidarCubeFaceDist + CUBE_HALF_EDGE;
	const auto lidarTransl = Vec3f{lidarCubeCenterDist, 0.0f, 0.0f};
	const rgl_mat3x4f lidarPose = Mat3x4f::TRS(lidarTransl).toRGL();

	// Scene
	spawnCubeOnScene(Mat3x4f::identity());

	constructMRGraph(raysTf, lidarPose, vlp16LidarHBeamDivergence, vlp16LidarVBeamDivergence, RGL_RETURN_FIRST_LAST);

	EXPECT_RGL_SUCCESS(rgl_graph_run(rays));

	// Verify the output
	const float epsilon = 1e-4f;

	const auto outPointcloud = TestPointCloud::createFromNode(format, fields);
	const auto isHits = outPointcloud.getFieldValues<IS_HIT_I32>();
	const auto points = outPointcloud.getFieldValues<XYZ_VEC3_F32>();
	const auto distances = outPointcloud.getFieldValues<DISTANCE_F32>();
	EXPECT_EQ(isHits.size(), 2); // Single beam, first and last return.
	EXPECT_EQ(points.size(), isHits.size());
	EXPECT_EQ(distances.size(), isHits.size());

	// First return.
	const auto expectedFirstPoint = Vec3f{CUBE_HALF_EDGE, 0.0f, 0.0f};
	EXPECT_TRUE(isHits.at(0));
	EXPECT_NEAR(distances.at(0), lidarCubeFaceDist, epsilon);
	EXPECT_NEAR(points.at(0).x(), expectedFirstPoint.x(), epsilon);
	EXPECT_NEAR(points.at(0).y(), expectedFirstPoint.y(), epsilon);
	EXPECT_NEAR(points.at(0).z(), expectedFirstPoint.z(), epsilon);

	// Second return.
	const float expectedDiameter = std::max(vlp16LidarHBeamDiameter, vlp16LidarVBeamDiameter);
	const auto expectedLastDistance = static_cast<float>(sqrt(pow(lidarCubeFaceDist, 2) + pow(expectedDiameter / 2, 2)));
	// Subtract because the ray is pointing as is the negative X axis.
	const auto expectedLastPoint = lidarTransl - Vec3f{expectedLastDistance, 0.0f, 0.0f};
	EXPECT_TRUE(isHits.at(1));
	EXPECT_NEAR(distances.at(1), expectedLastDistance, epsilon);
	EXPECT_NEAR(points.at(1).x(), expectedLastPoint.x(), epsilon);
	EXPECT_NEAR(points.at(1).y(), expectedLastPoint.y(), epsilon);
	EXPECT_NEAR(points.at(1).z(), expectedLastPoint.z(), epsilon);
}

#if RGL_BUILD_ROS2_EXTENSION
/**
 * This test verifies the performance of the multiple return feature in a dynamic scene
 * with two cubes placed one behind the other, one cube cyclically moving sideways.
 * LiDAR fires the beam in such a way that in some frames the beam partially overlaps the edge of the moving cube.
 */
TEST_F(GraphMultiReturn, cube_in_motion)
{
	/*
	 *                            gap
	 *               ________ <---->
	 *              |        |     |
	 *              |        |     |
	 *              |________|     |
	 *                             |
	 *                             X
	 *                           LIDAR
	 */

	GTEST_SKIP(); // Comment to run the test

	// Lidar
	const std::vector<rgl_mat3x4f> raysTf{Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, 90.0f}).toRGL()};
	const float lidarCubeFaceDist = 100.0f;
	const float lidarCubeCenterDist = lidarCubeFaceDist + CUBE_HALF_EDGE * 2;
	const Vec3f lidarTransl = {-lidarCubeCenterDist, 3.0f, 3.0f};
	const rgl_mat3x4f lidarPose = Mat3x4f::translation(lidarTransl).toRGL();

	// Scene
	const Vec2f gapRange = {0.001f, 0.5f};
	const std::vector<Mat3x4f> entitiesTransforms = {
	    Mat3x4f::TRS(Vec3f{-5.0f, lidarTransl.y() + gapRange.x() + CUBE_HALF_EDGE, lidarTransl.z()}),
	    Mat3x4f::TRS(Vec3f{0.0f, lidarTransl.y(), lidarTransl.z()}, {0, 0, 0}, {2, 2, 2})};
	std::vector<rgl_entity_t> entities = {spawnCubeOnScene(entitiesTransforms.at(0)),
	                                      spawnCubeOnScene(entitiesTransforms.at(1))};

	// Lidar with MR
	constructMRGraph(raysTf, lidarPose, vlp16LidarHBeamDivergence, vlp16LidarVBeamDivergence, RGL_RETURN_FIRST_LAST, true);

	// Camera
	const rgl_mat3x4f cameraPose = Mat3x4f::TRS(Vec3f{-8.0f, 1.0f, 5.0f}, {90.0f, 30.0f, -70.0f}).toRGL();
	constructCameraGraph(cameraPose);

	// Switching return modes
	constexpr int framesPerSwitch = 200;
	int frameId = 0;
	const auto switchToNextMode = [&]() {
		static int returnModeIdx = -1;
		static const std::vector<rgl_return_mode_t> returnModes = {RGL_RETURN_FIRST, RGL_RETURN_LAST, RGL_RETURN_FIRST_LAST};

		returnModeIdx = ++returnModeIdx % static_cast<int>(returnModes.size());
		EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_return_mode(raytrace, returnModes[returnModeIdx]));
	};

	while (true) {
		const auto newPose = (entitiesTransforms.at(0) *
		                      Mat3x4f::translation(0.0f, std::abs(2 * std::sin(frameId * 0.05f)) * gapRange.y() - 0.9f, 0.0f))
		                         .toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entities.at(0), &newPose));

		if (frameId++ % framesPerSwitch == 0) {
			switchToNextMode();
		}

		ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));
		ASSERT_RGL_SUCCESS(rgl_graph_run(rays));

		std::this_thread::sleep_for(50ms);
	}
}
#endif

#if RGL_BUILD_ROS2_EXTENSION
/**
 * This test verifies how changing the beam divergence affects the results obtained with the multi return feature enabled.
 * Three cubes arranged in the shape of a stairs are placed on the scene. LiDAR aims the only ray at the center of the middle “stair",
 * during the test the beam divergence angle is increased/decreased periodically.
 */
TEST_F(GraphMultiReturn, stairs)
{
	/*
	 *                    ____        ^
	 *               ____|            | Z+
	 *          ____| middle step     |
	 *         |                       ----> X+
	 */

	GTEST_SKIP(); // Comment to run the test

	// Scene
	const float stairsBaseHeight = 0.0f;
	const float stepWidth = 1.0f;
	const float stepHeight = vlp16LidarVBeamDiameter + 0.1f;
	const float stepDepth = 0.8f;
	const Vec3f stairsTranslation{2.0f, 0.0f, 0.0f};

	spawnStairsOnScene(stepWidth, stepHeight, stepDepth, stairsBaseHeight, stairsTranslation);

	// Lidar
	const std::vector<rgl_mat3x4f> raysTf{Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, 90.0f}).toRGL()};

	const float lidarMiddleStepDist = 1.5f * vlp16LidarObjectDistance;
	const Vec3f lidarTransl{-lidarMiddleStepDist + stepDepth, 0.0f, stepHeight * 1.5f};

	const rgl_mat3x4f lidarPose{(Mat3x4f::translation(lidarTransl + stairsTranslation)).toRGL()};

	// Lidar with MR
	constructMRGraph(raysTf, lidarPose, vlp16LidarHBeamDivergence, vlp16LidarVBeamDivergence, RGL_RETURN_FIRST_LAST, true);

	// Camera
	rgl_mat3x4f cameraPose = Mat3x4f::translation({0.0f, -1.5f, stepHeight * 3 + 1.0f}).toRGL();
	constructCameraGraph(cameraPose);
	ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));

	int frameId = 0;
	while (true) {
		const float newVBeamDivAngle = vlp16LidarVBeamDivergence + std::sin(frameId * 0.1f) * vlp16LidarVBeamDivergence;
		ASSERT_RGL_SUCCESS(rgl_node_raytrace_configure_beam_divergence(raytrace, vlp16LidarHBeamDivergence, newVBeamDivAngle));

		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

		std::this_thread::sleep_for(100ms);
		frameId += 1;
	}
}

/**
 * This test verifies the performance of the multi return feature when releasing multiple ray beams at once.
 */
TEST_F(GraphMultiReturn, multiple_ray_beams)
{
	GTEST_SKIP(); // Comment to run the test

	// Scene
	spawnCubeOnScene(Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f}));

	// Camera
	constructCameraGraph(Mat3x4f::identity().toRGL());

	// Lidar with MR
	const int horizontalSteps = 10;
	const auto resolution = 360.0f / horizontalSteps;
	std::vector<rgl_mat3x4f> vlp16RaysTf;
	vlp16RaysTf.reserve(horizontalSteps * vlp16Channels.size());

	for (int i = 0; i < horizontalSteps; ++i) {
		for (const auto& velodyneVLP16Channel : vlp16Channels) {
			vlp16RaysTf.emplace_back((Mat3x4f::rotationDeg(0.0f, 90.0f, resolution * i) * velodyneVLP16Channel).toRGL());
		}
	}
	const rgl_mat3x4f lidarPose = Mat3x4f::identity().toRGL();
	constructMRGraph(vlp16RaysTf, lidarPose, vlp16LidarHBeamDivergence, vlp16LidarVBeamDivergence, RGL_RETURN_FIRST_LAST, true);

	ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));
	ASSERT_RGL_SUCCESS(rgl_graph_run(rays));
}

/**
 * In the MR implementation, the beam is modeled using beam samples, from which the first and last hits are calculated.
 * This test uses the code that generates these beam samples and fires them inside the cube.
 * It verifies the accuracy of beam sample generation with different horizontal and vertical divergence angles.
 */
TEST_F(GraphMultiReturn, horizontal_vertical_beam_divergence)
{
	GTEST_SKIP(); // Comment to run the test

	const float hHalfDivergenceAngleRad = M_PI / 4;
	const float vHalfDivergenceAngleRad = M_PI / 8;

	// Lidar
	const auto lidarTransl = Vec3f{0.0f, 0.0f, 0.0f};
	const rgl_mat3x4f lidarPose = Mat3x4f::TRS(lidarTransl).toRGL();
	std::vector<rgl_mat3x4f> raysTf;
	raysTf.reserve(MULTI_RETURN_BEAM_SAMPLES);

	raysTf.emplace_back(Mat3x4f::identity().toRGL());

	// Code that generates beam samples in the makeBeamSampleRayTransform function (gpu/optixPrograms.cu)
	for (int layerIdx = 0; layerIdx < MULTI_RETURN_BEAM_LAYERS; ++layerIdx) {
		for (int vertexIdx = 0; vertexIdx < MULTI_RETURN_BEAM_VERTICES; ++vertexIdx) {
			const float hCurrentDivergence = hHalfDivergenceAngleRad *
			                                 (1.0f - static_cast<float>(layerIdx) / MULTI_RETURN_BEAM_LAYERS);
			const float vCurrentDivergence = vHalfDivergenceAngleRad *
			                                 (1.0f - static_cast<float>(layerIdx) / MULTI_RETURN_BEAM_LAYERS);

			const float angleStep = 2.0f * static_cast<float>(M_PI) / MULTI_RETURN_BEAM_VERTICES;

			const float hAngle = hCurrentDivergence * std::cos(static_cast<float>(vertexIdx) * angleStep);
			const float vAngle = vCurrentDivergence * std::sin(static_cast<float>(vertexIdx) * angleStep);

			const auto rotation = Mat3x4f::rotationRad(vAngle, 0.0f, 0.0f) * Mat3x4f::rotationRad(0.0f, hAngle, 0.0f);
			raysTf.emplace_back(rotation.toRGL());
		}
	}

	// Scene
	spawnCubeOnScene(Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {6.0f, 6.0f, 6.0f}));

	// Camera
	constructCameraGraph(Mat3x4f::identity().toRGL());

	// Graph without MR
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transform, &lidarPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&publish, "MRTest_Lidar_Without_MR", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, transform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transform, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, publish));

	ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));
	ASSERT_RGL_SUCCESS(rgl_graph_run(rays));
}
#endif
