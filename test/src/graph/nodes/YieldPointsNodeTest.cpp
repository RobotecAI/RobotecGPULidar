#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class YieldPointsNodeTest : public RGLTest {
protected:
    // TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields
    std::vector<rgl_field_t> fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
};

TEST_F(YieldPointsNodeTest, invalid_arguments)
{
    rgl_node_t yieldNode = nullptr;

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(nullptr, nullptr, 0), "node != nullptr");

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&yieldNode, nullptr, 0), "fields != nullptr");

    yieldNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&yieldNode, fields.data(), 0), "field_count > 0");
}

TEST_F(YieldPointsNodeTest, valid_arguments)
{
    rgl_node_t yieldNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));
    ASSERT_THAT(yieldNode, NotNull());

    // If (*yieldNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));
}
