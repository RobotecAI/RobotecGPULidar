#include <helpers/commonHelpers.hpp>
#include "helpers/lidarHelpers.hpp"
#include "helpers/testPointCloud.hpp"
#include "helpers/sceneHelpers.hpp"

#include "RGLFields.hpp"
#include "math/Mat3x4f.hpp"

using namespace std::chrono_literals;

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

class GraphMultiReturn : public RGLTest
{
protected:
	// VLP16 data
	const float vlp16LidarObjectDistance = 100.0f;
	const float vlp16LidarHBeamDivergence = 0.003f; // Velodyne VLP16 horizontal beam divergence in rads
	const float vlp16LidarHBeamDiameter = 0.2868f;  // Velodyne VLP16 beam horizontal diameter at 100m

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

	const std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, IS_HIT_I32, DISTANCE_F32 /*, INTENSITY_F32, ENTITY_ID_I32*/};

	rgl_node_t rays = nullptr, mrRays = nullptr, cameraRays = nullptr, transform = nullptr, mrTransform = nullptr,
	           cameraTransform = nullptr, raytrace = nullptr, mrRaytrace = nullptr, cameraRaytrace = nullptr, mrFirst = nullptr,
	           mrLast = nullptr, format = nullptr, mrFormatFirst = nullptr, mrFormatLast = nullptr, cameraFormat = nullptr,
	           publish = nullptr, cameraPublish = nullptr, mrPublishFirst = nullptr, mrPublishLast = nullptr,
	           compactFirst = nullptr, compactLast = nullptr;

	void constructMRGraph(const std::vector<rgl_mat3x4f>& raysTf, const rgl_mat3x4f& lidarPose, const float beamDivAngle,
	                      bool withPublish = false)
	{
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&mrRays, raysTf.data(), raysTf.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&mrTransform, &lidarPose));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&mrRaytrace, nullptr));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_beam_divergence(mrRaytrace, beamDivAngle));
		EXPECT_RGL_SUCCESS(rgl_node_multi_return_switch(&mrFirst, RGL_RETURN_TYPE_FIRST));
		EXPECT_RGL_SUCCESS(rgl_node_multi_return_switch(&mrLast, RGL_RETURN_TYPE_LAST));
		EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactFirst, RGL_FIELD_IS_HIT_I32));
		EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactLast, RGL_FIELD_IS_HIT_I32));
		EXPECT_RGL_SUCCESS(rgl_node_points_format(&mrFormatFirst, fields.data(), fields.size()));
		EXPECT_RGL_SUCCESS(rgl_node_points_format(&mrFormatLast, fields.data(), fields.size()));
#if RGL_BUILD_ROS2_EXTENSION
		if (withPublish) {
			EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&mrPublishFirst, "MRTest_First", "world"));
			EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&mrPublishLast, "MRTest_Last", "world"));
		}
#else
		if (withPublish) {
			GTEST_SKIP() << "Publishing is not supported without ROS2 extension. Skippping the test.";
		}
#endif
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrRays, mrTransform));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrTransform, mrRaytrace));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrRaytrace, mrFirst));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrRaytrace, mrLast));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrFirst, compactFirst));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrLast, compactLast));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactFirst, mrFormatFirst));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactLast, mrFormatLast));
		if (withPublish) {
			EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrFormatFirst, mrPublishFirst));
			EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(mrFormatLast, mrPublishLast));
		}
	}

	void constructCameraGraph(const rgl_mat3x4f& cameraPose)
	{
#ifdef RGL_BUILD_ROS2_EXTENSION
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

	float mmToMeters(float mm) const { return mm * 0.001f; }
};

/**
 * This test verifies the accuracy of multiple return handling for the data specified for LiDAR VLP16
 * by firing a single beam into a cube and making sure the first and last hits are correctly calculated.
 */
TEST_F(GraphMultiReturn, VLP16_data_compare)
{
	// Lidar
	const std::vector<rgl_mat3x4f> raysTf{Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, -90.0f}).toRGL()};
	const float lidarCubeFaceDist = vlp16LidarObjectDistance;
	const float lidarCubeCenterDist = lidarCubeFaceDist + CUBE_HALF_EDGE;
	const rgl_mat3x4f lidarPose = Mat3x4f::TRS({lidarCubeCenterDist, 0.0f, 0.0f}).toRGL();

	// Scene
	spawnCubeOnScene(Mat3x4f::identity());

	// VLP16 horizontal beam divergence in rads
	const float beamDivAngle = vlp16LidarHBeamDivergence;
	constructMRGraph(raysTf, lidarPose, beamDivAngle);

	EXPECT_RGL_SUCCESS(rgl_graph_run(mrRays));

	// Verify the output
	const float epsilon = 1e-5f;

	const auto mrFirstOutPointcloud = TestPointCloud::createFromNode(mrFormatFirst, fields);
	const auto mrFirstIsHits = mrFirstOutPointcloud.getFieldValues<IS_HIT_I32>();
	const auto mrFirstPoints = mrFirstOutPointcloud.getFieldValues<XYZ_VEC3_F32>();
	const auto mrFirstDistances = mrFirstOutPointcloud.getFieldValues<DISTANCE_F32>();
	const auto expectedFirstPoint = Vec3f{CUBE_HALF_EDGE, 0.0f, 0.0f};
	EXPECT_EQ(mrFirstOutPointcloud.getPointCount(), raysTf.size());
	EXPECT_TRUE(mrFirstIsHits.at(0));
	EXPECT_NEAR(mrFirstPoints.at(0).x(), expectedFirstPoint.x(), epsilon);
	EXPECT_NEAR(mrFirstPoints.at(0).y(), expectedFirstPoint.y(), epsilon);
	EXPECT_NEAR(mrFirstPoints.at(0).z(), expectedFirstPoint.z(), epsilon);
	EXPECT_NEAR(mrFirstDistances.at(0), lidarCubeFaceDist, epsilon);

	const float expectedDiameter = vlp16LidarHBeamDiameter;

	const auto mrLastOutPointcloud = TestPointCloud::createFromNode(mrFormatLast, fields);
	const auto mrLastIsHits = mrLastOutPointcloud.getFieldValues<IS_HIT_I32>();
	const auto mrLastPoints = mrLastOutPointcloud.getFieldValues<XYZ_VEC3_F32>();
	const auto mrLastDistances = mrLastOutPointcloud.getFieldValues<DISTANCE_F32>();
	const auto expectedLastPoint = Vec3f{CUBE_HALF_EDGE, 0.0f, -expectedDiameter / 2};
	EXPECT_EQ(mrLastOutPointcloud.getPointCount(), raysTf.size());
	EXPECT_TRUE(mrLastIsHits.at(0));
	EXPECT_NEAR(mrLastPoints.at(0).x(), expectedLastPoint.x(), epsilon);
	EXPECT_NEAR(mrLastPoints.at(0).y(), expectedLastPoint.y(), epsilon);
	EXPECT_NEAR(mrLastPoints.at(0).z(), expectedLastPoint.z(), 0.01f);
}

#if RGL_BUILD_ROS2_EXTENSION
/**
 * This test verifies the performance of the multiple return feature in a dynamic scene
 * with two cubes placed one behind the other, one cube cyclically moving sideways.
 * LiDAR fires the beam in such a way that in some frames the beam partially overlaps the edge of the moving cube.
 */
TEST_F(GraphMultiReturn, pairs_of_cubes_in_motion)
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
	const float beamDivAngle = 0.003f;
	rgl_node_t mrRays = nullptr, mrRaytrace = nullptr, mrFormatFirst = nullptr, mrFormatLast = nullptr;
	constructMRGraph(raysTf, lidarPose, beamDivAngle, true);

	// Lidar without MR
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transform, &lidarPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&publish, "MRTest_Lidar_Without_MR", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, transform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transform, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, publish));

	// Camera
	const rgl_mat3x4f cameraPose = Mat3x4f::TRS(Vec3f{-8.0f, 1.0f, 5.0f}, {90.0f, 30.0f, -70.0f}).toRGL();
	constructCameraGraph(cameraPose);

	int frameId = 0;
	while (true) {

		const auto newPose = (entitiesTransforms.at(0) *
		                      Mat3x4f::translation(0.0f, std::abs(std::sin(frameId * 0.05f)) * gapRange.y(), 0.0f))
		                         .toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entities.at(0), &newPose));

		ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));
		ASSERT_RGL_SUCCESS(rgl_graph_run(rays));
		ASSERT_RGL_SUCCESS(rgl_graph_run(mrRays));

		std::this_thread::sleep_for(50ms);
		frameId += 1;
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
	const float stepHeight = vlp16LidarHBeamDiameter + 0.1f;
	const float stepDepth = 0.8f;
	const Vec3f stairsTranslation{2.0f, 0.0f, 0.0f};

	spawnStairsOnScene(stepWidth, stepHeight, stepDepth, stairsBaseHeight, stairsTranslation);

	// Lidar
	const std::vector<rgl_mat3x4f> raysTf{Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, 90.0f}).toRGL()};

	const float lidarMiddleStepDist = 1.5f * vlp16LidarObjectDistance;
	const Vec3f lidarTransl{-lidarMiddleStepDist + stepDepth, 0.0f, stepHeight * 1.5f};

	const rgl_mat3x4f lidarPose{(Mat3x4f::translation(lidarTransl + stairsTranslation)).toRGL()};

	// Lidar with MR
	const float beamDivAngle = vlp16LidarHBeamDivergence;
	constructMRGraph(raysTf, lidarPose, beamDivAngle, true);

	// Camera
	rgl_mat3x4f cameraPose = Mat3x4f::translation({0.0f, -1.5f, stepHeight * 3 + 1.0f}).toRGL();
	constructCameraGraph(cameraPose);

	int frameId = 0;
	while (true) {
		const float newBeamDivAngle = beamDivAngle + std::sin(frameId * 0.1f) * beamDivAngle;
		ASSERT_RGL_SUCCESS(rgl_node_raytrace_configure_beam_divergence(mrRaytrace, newBeamDivAngle));

		ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));
		ASSERT_RGL_SUCCESS(rgl_graph_run(mrRaytrace));

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
	const float beamDivAngle = vlp16LidarHBeamDivergence;
	constructMRGraph(vlp16RaysTf, lidarPose, beamDivAngle, true);

	ASSERT_RGL_SUCCESS(rgl_graph_run(cameraRays));
	ASSERT_RGL_SUCCESS(rgl_graph_run(mrRays));
}
#endif
