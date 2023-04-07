#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class FormatPointsNodeTests : public RGLTest{ };

TEST_F(FormatPointsNodeTests, invalid_arguments)
{
    rgl_node_t formatNode = nullptr;
	// TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields.
	std::vector<rgl_field_t> fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, nullptr, 0), "fields != nullptr");

	formatNode = nullptr;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, fields.data(), 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTests, valid_arguments)
{
	rgl_node_t formatNode = nullptr;
	std::vector<rgl_field_t> fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};

	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
 	ASSERT_THAT(formatNode, NotNull());

	// If (*formatNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
}
