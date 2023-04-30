#include <utils.hpp>

class TemporalMergePointsNodeTest : public RGLTest{
protected:
    std::vector<rgl_field_t> fields;
    rgl_node_t temporalMergePointsNode;

    TemporalMergePointsNodeTest()
    {
        // TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields
        fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
        temporalMergePointsNode = nullptr;
    }
};

TEST_F(TemporalMergePointsNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(nullptr, fields.data(), fields.size()), "node != nullptr");
}

TEST_F(TemporalMergePointsNodeTest, invalid_argument_fields)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(&temporalMergePointsNode, nullptr, fields.size()), "fields != nullptr");
}

TEST_F(TemporalMergePointsNodeTest, invalid_argument_field_count)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(&temporalMergePointsNode, fields.data(), 0), "field_count > 0");
}

TEST_F(TemporalMergePointsNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMergePointsNode, fields.data(), fields.size()));
    ASSERT_THAT(temporalMergePointsNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMergePointsNode, fields.data(), fields.size()));
}
