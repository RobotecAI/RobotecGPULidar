#include <utils.hpp>

class FormatPointsNodeTests : public RGLTest{
protected:
    std::vector<rgl_field_t> fields;
    rgl_node_t formatNode;

    FormatPointsNodeTests() {
        fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
        formatNode = nullptr;
    }
};

// TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields.
TEST_F(FormatPointsNodeTests, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");
}

TEST_F(FormatPointsNodeTests, invalid_argument_fields)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, nullptr, 0), "fields != nullptr");
}

TEST_F(FormatPointsNodeTests, invalid_argument_field_count)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, fields.data(), 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTests, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    ASSERT_THAT(formatNode, testing::NotNull());

    // If (*formatNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
}
