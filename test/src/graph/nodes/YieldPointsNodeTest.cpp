#include <utils.hpp>

class YieldPointsNodeTest : public RGLTest {
protected:
    std::vector<rgl_field_t> fields;
    rgl_node_t yieldNode;

    YieldPointsNodeTest()
    {
        // TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields
        fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
        yieldNode = nullptr;
    }
};

TEST_F(YieldPointsNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(nullptr, nullptr, 0), "node != nullptr");
}

TEST_F(YieldPointsNodeTest, invalid_argument_fields)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&yieldNode, nullptr, 0), "fields != nullptr");
}

TEST_F(YieldPointsNodeTest, invalid_argument_fields_count)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&yieldNode, fields.data(), 0), "field_count > 0");
}

TEST_F(YieldPointsNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));
    ASSERT_THAT(yieldNode, testing::NotNull());

    // If (*yieldNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));
}
