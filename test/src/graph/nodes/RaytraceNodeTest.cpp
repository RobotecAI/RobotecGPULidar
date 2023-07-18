#include <scenes.hpp>
#include <utils.hpp>

class RaytraceNodeTest : public RGLTest {
protected:
    rgl_node_t raytraceNode;

    float raytraceDepth;

    RaytraceNodeTest()
    {
        raytraceNode = nullptr;
    }
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
    const rgl_mat3x4f identity = Mat3x4f::identity().toRGL();

    rgl_node_t raysFromMat = nullptr;
    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysFromMat, &identity, 1));

    raytraceDepth = 10.0f;
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, raytraceDepth));

    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysFromMat, raytraceNode));

    ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

    // The test fails. Important note:
    // When trying to take out the size of the results, an error is returned
    // Error message: Recoverable error (code=1): requested to reserve 0 elements
    int32_t hitpointCount, pointSize;
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytraceNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));

    // Also returned is an error from the rgl_cleanup()
    // Error message: Recoverable error (code=1): requested to reserve 0 elements

    // Potential solutions: 
    // warning that the scene is empty and performing a raytracing 
    // or error with the corresponding message that the scene is empty

    // rgl_cleanup() should also return a warning or error with a better specified comment
}
