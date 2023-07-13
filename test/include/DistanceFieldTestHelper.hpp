#include <random>
#include <scenes.hpp>
#include <utils.hpp>

struct DistanceFieldTestHelper {
    static constexpr float CUBE_HALF_EDGE = 1.0;

    static constexpr int ANGLE_ITERATIONS = 100;
    static constexpr int LIDAR_RAYS_COUNT = 100000;

    static constexpr float STD_DEV = 0.01;
    static constexpr float STD_DEV_PER_METER = 0.001;
    static constexpr float MEAN = 0.1;

    static constexpr float EPSILON_MUL = 2 * 1E-4;
    static constexpr float EPSILON_NOISE = 0.002;

    rgl_node_t useRaysNode = nullptr;
    rgl_node_t setRangeNode = nullptr;
    rgl_node_t raytraceNode = nullptr;
    rgl_node_t gaussianNoiseNode = nullptr;
    rgl_node_t yieldPointsNode = nullptr;
    rgl_node_t transformPointsNode = nullptr;

    rgl_mesh_t cubeMesh;
    rgl_entity_t cubeEntity = nullptr;

    std::vector<rgl_mat3x4f> rayTf;

    std::vector<rgl_field_t> fields = { DISTANCE_F32, XYZ_F32 };
    int32_t outDistancesCount, outDistancesSize, outPointsCount, outPointsSize;
    std::vector<::Field<DISTANCE_F32>::type> outDistances;
    std::vector<::Field<XYZ_F32>::type> outPoints;

    void prepareSceneWithCube(rgl_mat3x4f cubePoseTf = Mat3x4f::identity().toRGL())
    {
        cubeMesh = makeCubeMesh();
        EXPECT_RGL_SUCCESS(rgl_entity_create(&cubeEntity, nullptr, cubeMesh));
        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cubeEntity, &cubePoseTf));
    }

    void connectNodes(bool withGaussianNoiseNode)
    {
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, setRangeNode));
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(setRangeNode, raytraceNode));
        if (withGaussianNoiseNode) {
            ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, gaussianNoiseNode));
            ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(gaussianNoiseNode, yieldPointsNode));
        } else {
            ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldPointsNode));
        }
    }

    void disconnectNodes(bool withGaussianNoiseNode)
    {
        ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(useRaysNode, setRangeNode));
        ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(setRangeNode, raytraceNode));
        if (withGaussianNoiseNode) {
            ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(raytraceNode, gaussianNoiseNode));
            ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(gaussianNoiseNode, yieldPointsNode));
        } else {
            ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(raytraceNode, yieldPointsNode));
        }
    }

    void prepareNodes(rgl_vec2f raytraceRange)
    {
        ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
        ASSERT_RGL_SUCCESS(rgl_node_rays_set_range(&setRangeNode, &raytraceRange, 1));
        ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
        ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldPointsNode, fields.data(), fields.size()));
    }

    void getResults(rgl_node_t* node = nullptr)
    {
        ASSERT_RGL_SUCCESS(rgl_graph_run(useRaysNode));

        if (node == nullptr) {
            node = &yieldPointsNode;
        }
        ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(*node, DISTANCE_F32, &outDistancesCount, &outDistancesSize));
        outDistances.resize(outDistancesCount);
        ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(*node, DISTANCE_F32, outDistances.data()));
        ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(*node, XYZ_F32, &outPointsCount, &outPointsSize));
        outPoints.resize(outPointsCount);
        ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(*node, XYZ_F32, outPoints.data()));
    }
};