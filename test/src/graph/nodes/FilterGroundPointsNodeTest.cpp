#include <helpers/commonHelpers.hpp>
#include <helpers/mathHelpers.hpp>
#include "helpers/sceneHelpers.hpp"
#include "helpers/lidarHelpers.hpp"
#include "helpers/testPointCloud.hpp"

struct FilterGroundPointsNodeTest : public RGLTest
{
	static const inline auto WALL_POS = Vec3f(0, -10, 0);
	static const inline auto WALL_DIMS = Vec3f(200, 0.1, 200);
	static const inline rgl_vec3f UP_VEC = {0.0f, 1.0f, 0.0f};

	std::vector<rgl_field_t> basicFields = {XYZ_VEC3_F32, IS_HIT_I32};
	std::vector<rgl_field_t> allFields = {XYZ_VEC3_F32, IS_HIT_I32, IS_GROUND_I32};
	rgl_node_t filterGroundNode;

	FilterGroundPointsNodeTest() { filterGroundNode = nullptr; }
};

TEST_F(FilterGroundPointsNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_filter_ground(nullptr, &UP_VEC, 0.0f), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_filter_ground(&filterGroundNode, &UP_VEC, -1.0f),
	                            "ground_angle_threshold >= 0");
}

TEST_F(FilterGroundPointsNodeTest, valid_argument_node)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_filter_ground(&filterGroundNode, &UP_VEC, 1.0f));
}

TEST_F(FilterGroundPointsNodeTest, floor_test)
{
	// Scene
	rgl_entity_t wall = makeEntity(makeCubeMesh());
	rgl_mat3x4f wallPose = Mat3x4f::TRS(WALL_POS, Vec3f(0, 0, 0), WALL_DIMS).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(wall, &wallPose));

	// Rays
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

	// Graph
	rgl_node_t useRaysNode = nullptr;
	rgl_node_t raytraceNode = nullptr;
	rgl_node_t yieldNode = nullptr;

	// Prepare graph without filtering
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, basicFields.data(), basicFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
	TestPointCloud outputPointCloud = TestPointCloud::createFromNode(yieldNode, basicFields);
	auto fullCloudSize = outputPointCloud.getPointCount();

	// Prepare graph with non-hit filtering
	rgl_node_t compactNonHitNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(raytraceNode, yieldNode));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactNonHitNode, IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNonHitNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNonHitNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
	TestPointCloud hitPointCloud = TestPointCloud::createFromNode(yieldNode, basicFields);
	auto hitCloudSize = hitPointCloud.getPointCount();
	// Compacted cloud should be smaller and contain only hit points
	EXPECT_LT(hitCloudSize, fullCloudSize);

	// Prepare graph with ground filtering
	rgl_node_t groundFilterNode = nullptr;
	rgl_node_t compactGroundNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(compactNonHitNode, yieldNode));
	// Add ground filtering with ground compacting
	EXPECT_RGL_SUCCESS(rgl_node_points_filter_ground(&groundFilterNode, &UP_VEC, std::numbers::pi_v<float> / 2.0f));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactGroundNode, IS_GROUND_I32));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNonHitNode, groundFilterNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(groundFilterNode, compactGroundNode));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, allFields.data(), allFields.size()));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactGroundNode, yieldNode));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	TestPointCloud nonGroundPointCloud = TestPointCloud::createFromNode(yieldNode, allFields);
	auto nonGroundCloudSize = nonGroundPointCloud.getPointCount();

	// All points which were hit should be ground hits. In that case compacted cloud should be empty.
	EXPECT_EQ(nonGroundCloudSize, 0);
}