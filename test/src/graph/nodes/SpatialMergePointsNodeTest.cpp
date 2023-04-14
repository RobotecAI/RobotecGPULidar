#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

class SpatialMergePointsNodeTest : public RGLTest{
protected:
    // TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields
    std::vector<rgl_field_t> fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
};

TEST_F(SpatialMergePointsNodeTest, invalid_arguments)
{
    rgl_node_t spatialMergePointsNode = nullptr;

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(nullptr, fields.data(), fields.size()), "node != nullptr");

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(&spatialMergePointsNode, nullptr, fields.size()), "fields != nullptr");

    spatialMergePointsNode = nullptr;
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(&spatialMergePointsNode, fields.data(), 0), "field_count > 0");
}

TEST_F(SpatialMergePointsNodeTest, valid_arguments)
{
    rgl_node_t spatialMergePointsNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMergePointsNode, fields.data(), fields.size()));
    ASSERT_THAT(spatialMergePointsNode, testing::NotNull());

    // If (*raysFromMatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMergePointsNode, fields.data(), fields.size()));
}
