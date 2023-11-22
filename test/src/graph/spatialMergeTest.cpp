#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/lidarHelpers.hpp>

#include <math/Mat3x4f.hpp>
#include <Logger.hpp>

#if RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

class GraphSpatialMergeTest : public RGLTest
{};

TEST_F(GraphSpatialMergeTest, SpatialMergeFromTransforms)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, compact = nullptr;
	rgl_node_t transformPtsZero = nullptr, transformPtsY = nullptr;
	rgl_node_t spatialMerge = nullptr;
	std::vector<rgl_field_t> sMergeFields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32};

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPtsZero, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPtsY, &translateYTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPtsZero));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPtsY));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPtsZero, spatialMerge));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPtsY, spatialMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(spatialMerge, "two_boxes_spatial_merge.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

TEST_F(GraphSpatialMergeTest, SpatialMergeFromRaytraces)
{
	// Setup cube scene
	auto mesh = makeCubeMesh();
	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	constexpr int LIDAR_FOV_Y = 40;
	constexpr int LIDAR_ROTATION_STEP = LIDAR_FOV_Y / 2; // Make laser overlaps to validate merging

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(180, LIDAR_FOV_Y, 0.18, 1);

	// Lidars will be located in the cube center with different rotations covering all the space.
	std::vector<rgl_mat3x4f> lidarTfs;
	for (int i = 0; i < 360 / LIDAR_ROTATION_STEP; ++i) {
		lidarTfs.emplace_back(Mat3x4f::TRS({0, 0, 0}, {0, LIDAR_ROTATION_STEP * i, 0}).toRGL());
	}

	rgl_node_t spatialMerge = nullptr;
	std::vector<rgl_field_t> sMergeFields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32};
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	for (auto& lidarTf : lidarTfs) {
		rgl_node_t lidarRays = nullptr;
		rgl_node_t lidarRaysTf = nullptr;
		rgl_node_t raytrace = nullptr;

		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&lidarRays, rays.data(), rays.size()));
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarRaysTf, &lidarTf));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));

		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarRays, lidarRaysTf));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarRaysTf, raytrace));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, spatialMerge));
	}

	EXPECT_RGL_SUCCESS(rgl_graph_run(spatialMerge));
#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(spatialMerge, "cube_spatial_merge.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}