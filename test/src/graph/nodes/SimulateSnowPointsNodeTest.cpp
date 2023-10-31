#include "helpers/lidarHelpers.hpp"
#include "helpers/sceneHelpers.hpp"
#include <helpers/commonHelpers.hpp>

class SimulateSnowPointsNodeTest : public RGLTest
{};

TEST_F(SimulateSnowPointsNodeTest, invalid_argument_node)
{
	rgl_node_t snowfallNode = nullptr;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_simulate_snow(nullptr, 100.0f, 1.0f, 0.001f, 1.0f, 0.1f, 10, 0.01f),
	                            "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_simulate_snow(&snowfallNode, 100.0f, 1.0f, 0.001f, 1.0f, 0.1f, 0, 0.01f),
	                            "num_channels > 0");
}

TEST_F(SimulateSnowPointsNodeTest, valid_argument_node)
{
	rgl_node_t simulateSnowNode = nullptr;
	rgl_configure_logging(RGL_LOG_LEVEL_ALL, nullptr, true);
	EXPECT_RGL_SUCCESS(rgl_node_points_simulate_snow(&simulateSnowNode, 100.0f, 9.0f, 0.0002f, 1.6f, 0.07f, 10, 0.01f));
}

TEST_F(SimulateSnowPointsNodeTest, use_case)
{
	rgl_node_t useRays = nullptr;
	rgl_node_t raytrace = nullptr;
	rgl_node_t compact = nullptr;
	rgl_node_t simulateSnow = nullptr;
	rgl_configure_logging(RGL_LOG_LEVEL_ALL, nullptr, true);
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_simulate_snow(&simulateSnow, 100.0f, 9.0f, 0.0002f, 1.6f, 0.07f, 10, 0.01f));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, simulateSnow));

	EXPECT_RGL_SUCCESS(rgl_graph_run(useRays));
}