#include <helpers/commonHelpers.hpp>
#include <helpers/mathHelpers.hpp>
#include "helpers/sceneHelpers.hpp"
#include "helpers/lidarHelpers.hpp"

struct FilterGroundPointsNodeTest : public RGLTest
{
	static const inline auto WALL_POS = Vec3f(0, 0, -10.0f);
	static const inline auto WALL_DIMS = Vec3f(200, 0.1, 200);
protected:
	rgl_node_t filterGroundNode;

	FilterGroundPointsNodeTest() { filterGroundNode = nullptr; }
};

TEST_F(FilterGroundPointsNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_filter_ground(nullptr, RGL_AXIS_Y, 0.0f), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_filter_ground(&filterGroundNode, RGL_AXIS_Y, -1.0f), "ground_angle_threshold >= 0");
}

TEST_F(FilterGroundPointsNodeTest, valid_argument_node)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_filter_ground(&filterGroundNode, RGL_AXIS_Y, 1.0f));
}

TEST_F(FilterGroundPointsNodeTest, floor_test)
{
	// Scene
	rgl_entity_t wall = makeEntity(makeCubeMesh());
	rgl_mat3x4f wallPose = Mat3x4f::TRS(WALL_POS , Vec3f(0, 0, 0), WALL_DIMS).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(wall, &wallPose));

	// Rays
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

	// Graph
	rgl_node_t useRaysNode = nullptr;
	rgl_node_t transformRaysNode = nullptr;
	rgl_node_t raytraceNode = nullptr;
	rgl_node_t groundFilterNode = nullptr;
	rgl_node_t compactNode = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRaysNode, &identityTestTransform));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_filter_ground(&groundFilterNode, RGL_AXIS_Y, 0.0f));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, transformRaysNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, groundFilterNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(groundFilterNode, compactNode));

	// Run graph
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	// Check results


}