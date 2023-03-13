#include <helpers/commonHelpers.hpp>

class RaytraceNodeTest : public RGLTest {
protected:
    rgl_node_t raytraceNode;

    RaytraceNodeTest()
    {
        raytraceNode = nullptr;
    }
};

TEST_F(RaytraceNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr), "node != nullptr");
}

TEST_F(RaytraceNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
    ASSERT_THAT(raytraceNode, testing::NotNull());

    // If (*raytraceNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
}
