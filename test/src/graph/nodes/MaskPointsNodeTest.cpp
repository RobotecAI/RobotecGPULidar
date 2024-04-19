#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>
#include <RGLFields.hpp>

class MaskPointsNodeTest : public RGLTest
{

protected:
	int pointsCount;
	std::vector<int32_t> points_mask;
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, IS_HIT_I32, INTENSITY_F32, IS_GROUND_I32};

	void initializeMask(int pointCount, int maskCount)
	{
		points_mask.resize(pointsCount);
		std::fill(points_mask.begin(), points_mask.end(), 1);
		for (int i = 0; i < maskCount; i++) {
			points_mask[i] = 0;
		}
	}
};

TEST_F(MaskPointsNodeTest, invalid_argument_node) { EXPECT_RGL_INVALID_ARGUMENT(rgl_node_mask_points(nullptr, nullptr, 0)); }

TEST_F(MaskPointsNodeTest, invalid_argument_mask)
{
	rgl_node_t maskPointsNode;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_mask_points(&maskPointsNode, nullptr, 0));
}

TEST_F(MaskPointsNodeTest, invalid_argument_count)
{
	rgl_node_t maskPointsNode;
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_mask_points(&maskPointsNode, points_mask.data(), 0));
}

TEST_F(MaskPointsNodeTest, valid_arguments)
{
	rgl_node_t maskPointsNode = nullptr;
	initializeMask(pointsCount, 100);
	EXPECT_RGL_SUCCESS(rgl_node_mask_points(&maskPointsNode, points_mask.data(), 1));
	ASSERT_THAT(maskPointsNode, testing::NotNull());
}

TEST_F(MaskPointsNodeTest, use_case)
{
	TestPointCloud pointCloud = TestPointCloud(fields, pointsCount);
	pointCloud.removeNonHitPoints();
	pointsCount = pointCloud.getPointCount();
	int maskCount = pointsCount / 2;

	initializeMask(pointsCount, maskCount);

	rgl_node_t usePointsNode = pointCloud.createUsePointsNode();
	rgl_node_t maskPointsNode = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_mask_points(&maskPointsNode, points_mask.data(), pointsCount));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, maskPointsNode));

	rgl_node_t compactNode = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactNode, IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(maskPointsNode, compactNode));

	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	TestPointCloud outputPointCloud = TestPointCloud::createFromNode(compactNode, fields);

	ASSERT_EQ(outputPointCloud.getPointCount(), pointsCount - maskCount);
}