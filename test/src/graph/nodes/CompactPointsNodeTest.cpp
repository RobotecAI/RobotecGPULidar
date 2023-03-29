#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class CompactPointsNodeTest : public RGLTest{ };

TEST_F(CompactPointsNodeTest, invalid_arguments)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "node != nullptr");
}

TEST_F(CompactPointsNodeTest, valid_arguments)
{
    rgl_node_t compactNode = nullptr;

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    EXPECT_THAT(compactNode, NotNull());

    // If (*compactNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
}
