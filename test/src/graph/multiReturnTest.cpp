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
	const std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, IS_HIT_I32, DISTANCE_F32 /*, INTENSITY_F32, ENTITY_ID_I32*/};

	rgl_node_t rays = nullptr, mrRays = nullptr, cameraRays = nullptr, transform = nullptr, mrTransform = nullptr,
	           cameraTransform = nullptr, raytrace = nullptr, mrRaytrace = nullptr, cameraRaytrace = nullptr, mrFirst = nullptr,
	           mrLast = nullptr, format = nullptr, mrFormatFirst = nullptr, mrFormatLast = nullptr, cameraFormat = nullptr,
	           publish = nullptr, cameraPublish = nullptr, mrPublishFirst = nullptr, mrPublishLast = nullptr,
	           compactFirst = nullptr, compactLast = nullptr;

	void constructMRGraph(const std::vector<rgl_mat3x4f>& raysTf, const rgl_mat3x4f& lidarPose, const float beamDivAngle,
	                      bool withPublish)
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
};

/**
 * This test verifies the accuracy of multiple return handling for the data specified for LiDAR VLP16
 * by firing a single beam into a cube and making sure the first and last hits are correctly calculated.
 */
TEST_F(GraphMultiReturn, VLP16_data_compare)
{
	// Lidar
	const std::vector<rgl_mat3x4f> raysTf {Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, -90.0f}).toRGL()};
	const float lidarCubeFaceDist = 100.0f;
	const float lidarCubeCenterDist = lidarCubeFaceDist + CUBE_HALF_EDGE;
	const rgl_mat3x4f lidarPose = Mat3x4f::TRS({lidarCubeCenterDist, 0.0f, 0.0f}).toRGL();

	// Scene
	spawnCubeOnScene(Mat3x4f::identity());

	// VLP16 horizontal beam divergence in rads
	const float beamDivAngle = 0.003f;
	constructMRGraph(raysTf, lidarPose, beamDivAngle, false);

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

	const float expectedDiameter = 0.2868f; // VLP16 beam horizontal diameter at 100m

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
	const std::vector<rgl_mat3x4f> raysTf {Mat3x4f::TRS({0.0f, 0.0f, 0.0f}, {90.0f, 0.0f, 90.0f}).toRGL()};
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

	// Camera
	const rgl_mat3x4f cameraPose = Mat3x4f::TRS(Vec3f{-8.0f, 1.0f, 5.0f}, {90.0f, 30.0f, -70.0f}).toRGL();
	const std::vector<rgl_mat3x4f> cameraRayTf = makeLidar3dRays(360.0f, 180.0f, 0.5f, 0.5f);
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&cameraRays, cameraRayTf.data(), cameraRayTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&cameraTransform, &cameraPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&cameraRaytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&cameraFormat, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&cameraPublish, "MRTest_PairsOfCubes_Camera", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraRays, cameraTransform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraTransform, cameraRaytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraRaytrace, cameraFormat));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraFormat, cameraPublish));

	// Lidar without MR
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transform, &lidarPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&publish, "MRTest_PairsOfCubes", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, transform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transform, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, publish));

	// Lidar with MR
	const float beamDivAngle = 0.003f;
	constructMRGraph(raysTf, lidarPose, beamDivAngle, true);

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