#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class FormatPointsNodeTests : public RGLAutoCleanupTest{ };

TEST_F(FormatPointsNodeTests, invalid_arguments)
{
    rgl_node_t formatNode = nullptr;
	rgl_field_t field = RGL_FIELD_INTENSITY_F32;

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, nullptr, 0), "fields != nullptr");

	formatNode = nullptr;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, &field, 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTests, valid_arguments)
{
	rgl_node_t formatNode = nullptr;
	rgl_field_t field = RGL_FIELD_INTENSITY_F32;

	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, &field, 1));
 	ASSERT_THAT(formatNode, NotNull());
}
