#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/lidarHelpers.hpp>

#include <math/Mat3x4f.hpp>
#include <Logger.hpp>

#if RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

class GraphNodeRemovalTest : public RGLTest
{};

TEST_F(GraphNodeRemovalTest, NodeRemoval)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_transform(entity, &entityPoseTf));

	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, transformPts = nullptr, compact = nullptr,
	           downsample = nullptr;
	rgl_node_t temporalMerge = nullptr;
	std::vector<rgl_field_t> tMergeFields = {RGL_FIELD_XYZ_VEC3_F32};

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateXTf = Mat3x4f::TRS({3, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, temporalMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Remove compact<->transformPts connection
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateXTf));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Restore compact<->transformPts connection
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateYTf));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Output pointcloud should contain two boxes
#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(temporalMerge, "two_boxes_removal.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}