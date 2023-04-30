#include <utils.hpp>

class SpatialMergePointsNodeTest : public RGLTest{
protected:
    std::vector<rgl_field_t> fields;
    rgl_node_t spatialMergePointsNode;

    SpatialMergePointsNodeTest()
    {
        fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
        spatialMergePointsNode = nullptr;
    }
};

TEST_F(SpatialMergePointsNodeTest, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(nullptr, fields.data(), fields.size()), "node != nullptr");
}

TEST_F(SpatialMergePointsNodeTest, invalid_argument_fields)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(&spatialMergePointsNode, nullptr, fields.size()), "fields != nullptr");
}

TEST_F(SpatialMergePointsNodeTest, invalid_argument_field_count)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(&spatialMergePointsNode, fields.data(), 0), "field_count > 0");
}

TEST_F(SpatialMergePointsNodeTest, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMergePointsNode, fields.data(), fields.size()));
    ASSERT_THAT(spatialMergePointsNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMergePointsNode, fields.data(), fields.size()));
}
