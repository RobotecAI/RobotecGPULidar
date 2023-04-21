#include <utils.hpp>

class CompactPointsNodeTest : public RGLTest{
protected:
    rgl_node_t compactNode;

    CompactPointsNodeTest() {
        compactNode = nullptr;
    }
};

TEST_F(CompactPointsNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "node != nullptr");
}

TEST_F(CompactPointsNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_THAT(compactNode, testing::NotNull());

    // If (*compactNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
}
