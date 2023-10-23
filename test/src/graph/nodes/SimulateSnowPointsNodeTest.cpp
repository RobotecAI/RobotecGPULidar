#include "helpers/lidarHelpers.hpp"
#include "helpers/sceneHelpers.hpp"
#include <helpers/commonHelpers.hpp>

class SimulateSnowPointsNodeTest : public RGLTest {

};

TEST_F(SimulateSnowPointsNodeTest, invalid_argument_node)
{
    rgl_node_t gaussianNoiseNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_simulate_snow(nullptr, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f), "node != nullptr");

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_simulate_snow(&gaussianNoiseNode, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f), "num_channels>0");
}

TEST_F(SimulateSnowPointsNodeTest, valid_argument_node)
{
    rgl_node_t simulateSnowNode = nullptr;
    EXPECT_RGL_SUCCESS(rgl_node_points_simulate_snow(&simulateSnowNode, 0.0f, 0.0f, 8, 0.0f, 0.0f));
}

TEST_F(SimulateSnowPointsNodeTest, use_case)
{
    rgl_node_t useRays = nullptr;
    rgl_node_t raytrace =nullptr;
    rgl_node_t compact = nullptr;
    rgl_node_t simulateSnow = nullptr;
    rgl_configure_logging(RGL_LOG_LEVEL_ALL,nullptr, true);
    std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
    EXPECT_RGL_SUCCESS(rgl_node_points_simulate_snow(&simulateSnow, 0.0f, 0.0f, 8, 1000.0f, 0.0f));


    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, simulateSnow));

    EXPECT_RGL_SUCCESS(rgl_graph_run(useRays));


}