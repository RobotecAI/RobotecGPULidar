#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/mathHelpers.hpp>

#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <repr.hpp>

class TransformPointsNodeTest : public RGLTestWithParam<std::tuple<int, rgl_mat3x4f>>
{
protected:
	static constexpr int TEST_PARAM_TRANSFORM_ID = 1;

	std::vector<rgl_field_t> fields = {XYZ_F32, IS_HIT_I32, INTENSITY_F32};
};

INSTANTIATE_TEST_SUITE_P(TransformPointsNodeTests, TransformPointsNodeTest,
                         testing::Combine(testing::Values(1, 100, maxGPUCoresTestCount),
                                          testing::Values(identityTestTransform, translationTestTransform,
                                                          rotationTestTransform, scalingTestTransform, complexTestTransform)));

TEST_P(TransformPointsNodeTest, invalid_arguments)
{
	rgl_mat3x4f transform;
	rgl_node_t transformPointsNode;

	auto initializeArgumentsLambda = [&transform, &transformPointsNode]() {
		transform = std::get<TEST_PARAM_TRANSFORM_ID>(GetParam());
		transformPointsNode = nullptr;
	};

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(nullptr, nullptr), "node != nullptr");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(nullptr, &transform), "node != nullptr");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(&transformPointsNode, nullptr), "transform != nullptr");
}

TEST_P(TransformPointsNodeTest, valid_arguments)
{
	auto [_, transform] = GetParam();
	rgl_node_t transformPointsNode = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPointsNode, &transform));
	ASSERT_THAT(transformPointsNode, testing::NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPointsNode, &transform));
}

TEST_P(TransformPointsNodeTest, use_case)
{
	auto [pointsCount, transform] = GetParam();
	rgl_node_t transformNode = nullptr;

	TestPointCloud pointCloud = TestPointCloud(fields, pointsCount);
	rgl_node_t usePointsNode = pointCloud.createUsePointsNode();

	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformNode, &transform));
	ASSERT_THAT(transformNode, testing::NotNull());

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, transformNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	TestPointCloud outputPointCloud = TestPointCloud::createFromNode(transformNode, fields);

	pointCloud.transform(Mat3x4f::fromRGL(transform));

	checkIfNearEqual(pointCloud.getFieldValues<XYZ_F32>(), outputPointCloud.getFieldValues<XYZ_F32>(), EPSILON_F);
	checkIfNearEqual(pointCloud.getFieldValues<IS_HIT_I32>(), outputPointCloud.getFieldValues<IS_HIT_I32>(), EPSILON_F);
	checkIfNearEqual(pointCloud.getFieldValues<INTENSITY_F32>(), outputPointCloud.getFieldValues<INTENSITY_F32>(), EPSILON_F);

	// TODO When we are testing big values of point translation, numerical errors appears.
	//  For example for 100000 unit of translation, error after rotation can extend 0.001 unit.
	//  To investigate in better times.
}