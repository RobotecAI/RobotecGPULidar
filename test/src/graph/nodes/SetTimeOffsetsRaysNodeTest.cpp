#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include "utils.hpp"


class SetTimeOffsetsRaysNodeTest : public RGLTest, public RGLPointTestHelper {};

TEST_F(SetTimeOffsetsRaysNodeTest, invalid_arguments)
{
	rgl_node_t setTimeOffsetsRaysNode;
	const std::vector<float> dummyOffsets(10);

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_time_offsets(nullptr, nullptr, 0), "node != nullptr");

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_time_offsets(&setTimeOffsetsRaysNode, nullptr, 0), "offsets != nullptr");

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_time_offsets(&setTimeOffsetsRaysNode, dummyOffsets.data(), -1), "offsets_count > 0");
}

TEST_F(SetTimeOffsetsRaysNodeTest, valid_arguments)
{
	int numberOfOffsets = 10;
	rgl_node_t setTimeOffsetsRaysNode = nullptr;
	std::vector<float> timeOffsets = generateTimeOffsetsForRays(numberOfOffsets);

	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&setTimeOffsetsRaysNode, timeOffsets.data(), numberOfOffsets));
	ASSERT_THAT(setTimeOffsetsRaysNode, testing::NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_time_offsets(&setTimeOffsetsRaysNode, timeOffsets.data(), numberOfOffsets));
}

TEST_F(SetTimeOffsetsRaysNodeTest, use_case)
{
	//TODO: implement
}


