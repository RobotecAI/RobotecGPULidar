#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class RaytraceNodeTest : public RGLTest { };

TEST_F(RaytraceNodeTest, invalid_arguments)
{
    rgl_node_t raytraceNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr, 0.0f), "node != nullptr");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(&raytraceNode, nullptr, 0.0f), "range > 0.0f");
}

TEST_F(RaytraceNodeTest, valid_arguments)
{
    rgl_node_t raytraceNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1.0f));
    ASSERT_THAT(raytraceNode, NotNull());

    // If (*raytraceNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1.0f));
}
