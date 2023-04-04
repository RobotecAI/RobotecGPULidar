#include "lidars.hpp"
#include "scenes.hpp"
#include "utils.hpp"

class InstanceIDTest : public RGLTest
{

};

TEST_F(InstanceIDTest, BaseTest)
{
    setupBoxesAlongAxes(nullptr);

    rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr;

    std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);


    EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));

    EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
}