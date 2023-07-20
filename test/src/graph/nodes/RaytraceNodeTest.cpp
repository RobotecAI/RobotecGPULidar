#include <scenes.hpp>
#include <utils.hpp>
#include <RGLFields.hpp>

static constexpr float CUBE_HALF_EDGE = 1.0;

class RaytraceNodeTest : public RGLTest {
protected:
    RaytraceNodeTest()
    {
        raytraceNode = nullptr;
        raysFromMatNode = nullptr;
    }

    void prepareScene()
    {
        const Mat3x4f cubeTf = Mat3x4f::translation(0, 0, cubePosition.z());
        spawnCubeOnScene(nullptr, cubeTf);
    }

    void prepareGraph()
    {
        ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMatNode, &identityTf, 1)); // TODO change hardcoded size
        ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, raytraceDepth));
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysFromMatNode, raytraceNode));
    }

    void getResults()
    {
        ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

        ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(raytraceNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));
        ASSERT_EQ(pointCount, 1);
        ASSERT_EQ(pointSize, sizeof(Vec3f));

        ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(raytraceNode, RGL_FIELD_XYZ_F32, &point));
    }

    Vec3f cubePosition = Vec3f(0, 0, 55.5f);
    rgl_mat3x4f identityTf = Mat3x4f::identity().toRGL();

    rgl_node_t raytraceNode, raysFromMatNode;
    float raytraceDepth;
    int32_t pointCount, pointSize;
    Vec3f point;
};

TEST_F(RaytraceNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr, 0.0f), "node != nullptr");
}

TEST_F(RaytraceNodeTest, invalid_argument_range)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(&raytraceNode, nullptr, 0.0f), "range > 0.0f");
}

TEST_F(RaytraceNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1.0f));
    ASSERT_THAT(raytraceNode, testing::NotNull());

    // If (*raytraceNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1.0f));
}

TEST_F(RaytraceNodeTest, empty_scene)
{
    /* 
    raytraceDepth = 10.0f;
    prepareGraph();

    // When trying to take out the size of the results, an error is returned
    // Error message: Recoverable error (code=1): requested to reserve 0 elements
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytraceNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));

    // Also returned is an error from the rgl_cleanup()
    // Error message: Recoverable error (code=1): requested to reserve 0 elements
    */

    // The test fails. It is commented because failure of rgl_cleanup affects other tests
    FAIL();

    // Potential solutions: 
    // warning that the scene is empty and performing a raytracing 
    // or error with the corresponding message that the scene is empty
    // rgl_cleanup() should also return a warning or error with a better specified comment
}

TEST_F(RaytraceNodeTest, raytrace_misses_object_when_depth_is_shorter_than_distance)
{
    raytraceDepth = cubePosition.z() - CUBE_HALF_EDGE - 1.0f;
    prepareScene();
    prepareGraph();
    getResults();

    EXPECT_EQ(point.x(), NON_HIT_VALUE);
    EXPECT_EQ(point.y(), NON_HIT_VALUE);
    EXPECT_EQ(point.z(), NON_HIT_VALUE);
}

TEST_F(RaytraceNodeTest, raytrace_hits_object_when_depth_equals_distance)
{
    raytraceDepth = cubePosition.z() - CUBE_HALF_EDGE;
    prepareScene();
    prepareGraph();
    getResults();

    EXPECT_EQ(point.x(), cubePosition.x());
    EXPECT_EQ(point.y(), cubePosition.y());
    EXPECT_EQ(point.z(), cubePosition.z() - CUBE_HALF_EDGE);
}

TEST_F(RaytraceNodeTest, raytrace_hits_object_when_depth_is_longer_than_distance)
{
    raytraceDepth = cubePosition.z() - CUBE_HALF_EDGE + 1.0f;
    prepareScene();
    prepareGraph();
    getResults();

    EXPECT_EQ(point.x(), cubePosition.x());
    EXPECT_EQ(point.y(), cubePosition.y());
    EXPECT_EQ(point.z(), cubePosition.z() - CUBE_HALF_EDGE);
}
