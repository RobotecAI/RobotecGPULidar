#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>

#include <RGLFields.hpp>
#include <graph/Node.hpp>

class FromArrayPointsNodeTest : public RGLTestWithParam<int> {
protected:
	rgl_node_t usePointsNode = nullptr;
	std::unique_ptr<TestPointCloud> pointCloud;
	std::vector<rgl_field_t> pointFields = {XYZ_F32, IS_HIT_I32, INTENSITY_F32};
};

INSTANTIATE_TEST_SUITE_P(
    FromArrayPointsNodeTests, FromArrayPointsNodeTest,
    testing::Values(1, 100, maxGPUCoresTestCount),
    [](const auto& info) {
        return "pointsCount_" + std::to_string(info.param);
    });

TEST_P(FromArrayPointsNodeTest, invalid_arguments)
{
    auto initializeArgumentsLambda = [this]() {
        int pointsCount = GetParam();
		pointCloud = std::make_unique<TestPointCloud>(pointFields, pointsCount);
		usePointsNode = nullptr;
    };

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, pointCloud->getData(), pointCloud->getPointCount(), pointFields.data(), 0), "field_count > 0");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, pointCloud->getData(), pointCloud->getPointCount(), nullptr, pointFields.size()), "fields != nullptr");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, pointCloud->getData(), 0, pointFields.data(), pointFields.size()), "points_count > 0");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, nullptr, pointCloud->getPointCount(), pointFields.data(), pointFields.size()), "points != nullptr");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(nullptr, pointCloud->getData(), pointCloud->getPointCount(), pointFields.data(), pointFields.size()), "node != nullptr");
}
TEST_P(FromArrayPointsNodeTest, valid_arguments)
{
    int pointsCount = GetParam();
	pointCloud = std::make_unique<TestPointCloud>(pointFields, pointsCount);

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, pointCloud->getData(), pointCloud->getPointCount(), pointFields.data(), pointFields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());
}

TEST_P(FromArrayPointsNodeTest, use_case)
{
    int pointsCount = GetParam();

	pointCloud = std::make_unique<TestPointCloud>(pointFields, pointsCount);

	usePointsNode = pointCloud->createUsePointsNode();
    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	TestPointCloud outputPointCloud = TestPointCloud::createFromNode(usePointsNode, pointFields);

	checkIfNearEqual(pointCloud->getFieldValues<XYZ_F32>(), outputPointCloud.getFieldValues<XYZ_F32>(), EPSILON_F);
	checkIfNearEqual(pointCloud->getFieldValues<IS_HIT_I32>(), outputPointCloud.getFieldValues<IS_HIT_I32>(), EPSILON_F);
	checkIfNearEqual(pointCloud->getFieldValues<INTENSITY_F32>(), outputPointCloud.getFieldValues<INTENSITY_F32>(), EPSILON_F);
}
