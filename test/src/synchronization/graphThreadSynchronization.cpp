#include "lidars.hpp"
#include "utils.hpp"
#include <scenes.hpp>
class graphThreadSynchronization : public RGLTestWithParam<int> { };

struct LinearGraph {

    rgl_node_t useRaysNode = nullptr;
    rgl_node_t transformRaysNode = nullptr;
    rgl_node_t raytraceNode = nullptr;
    rgl_node_t compactNode = nullptr;
    rgl_node_t transformPointsNode = nullptr;

    rgl_node_t transformAfterRunNode = nullptr;

    rgl_mat3x4f lidarPoseTf;
    rgl_mat3x4f shearTf;

    LinearGraph()
    {
        setupBoxesAlongAxes(nullptr);
        useRaysNode = nullptr, raytraceNode = nullptr, transformRaysNode = nullptr, transformPointsNode = nullptr, compactNode = nullptr;
        std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
        lidarPoseTf = Mat3x4f::TRS({ 5, 5, 5 }, { 45, 45, 45 }).toRGL();
        shearTf = Mat3x4f::shear({ 0, 0 }, { -1, -1 }, { 0, 0 }).toRGL();

        EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
        EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformRaysNode, &lidarPoseTf));
        EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
        EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
        EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPointsNode, &shearTf));

        EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, transformRaysNode));
        EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformRaysNode, raytraceNode));
        EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
        EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, transformPointsNode));
    }

    void RunGraph()
    {
        EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
    }

    void addNodeAfterRun()
    {
        EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformAfterRunNode, &identityTestTransform));
        EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPointsNode, transformAfterRunNode));
    }
};

INSTANTIATE_TEST_SUITE_P(graphThreadSynchronization, graphThreadSynchronization, testing::Values(1, 10, 1000));

TEST_P(graphThreadSynchronization, LinearMultipleRuns)
{
    int numberOfGraphIterations = GetParam();
    LinearGraph graph {};

    for (int i = 0; i < numberOfGraphIterations; ++i) {
        graph.RunGraph();
    }
}

TEST_F(graphThreadSynchronization, AddNodeAfterRun)
{
    LinearGraph graph {};
    graph.RunGraph();

    graph.addNodeAfterRun();

    graph.RunGraph();
}
// TODO(nebraszka): Write more unit test: trying to remove, add nodes when running is queued, accesing data, created, or not, etc.
