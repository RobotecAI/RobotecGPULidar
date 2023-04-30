#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <repr.hpp>
#include <utils.hpp>
#include <PointsTestHelper.hpp>

class TransformPointsNodeTest : public RGLTestWithParam<std::tuple<int, rgl_mat3x4f>>, public RGLPointsNodeTestHelper {
protected:
    static constexpr int TEST_PARAM_TRANSFORM_ID=1;
};

INSTANTIATE_TEST_SUITE_P(
    TransformPointsNodeTests, TransformPointsNodeTest,
    testing::Combine(
        testing::Values(1, 100, maxGPUCoresTestCount),
        testing::Values(identityTestTransform, translationTestTransform, rotationTestTransform, scalingTestTransform, complexTestTransform)));

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

    createTestUsePointsNode(pointsCount);

    EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformNode, &transform));
    ASSERT_THAT(transformNode, testing::NotNull());

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, transformNode));

    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    int32_t outCount, outSizeOf;
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(transformNode, XYZ_F32, &outCount, &outSizeOf));

    EXPECT_EQ(outCount, inPoints.size());
    EXPECT_EQ(outSizeOf, getFieldSize(XYZ_F32));

    std::vector<::Field<XYZ_F32>::type> outData;
    outData.resize(outCount);
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(transformNode, XYZ_F32, outData.data()));

    std::vector<TestPointStruct> expectedPoints = generateTestPointsArray(outCount, transform);

    // TODO When we are testing big values of point translation, numerical errors appears.
    //  For example for 100000 unit of translation, error after rotation can extend 0.001 unit.
    //  To investigate in better times.

    // TODO 2 In the future, this and other tests, should check if every possible field has proper value, even if intact.
    for (int i = 0; i < pointsCount; ++i) {
        // For now use percent (0.1%) of value as error treshold.
        EXPECT_NEAR(expectedPoints.at(i).xyz[0], outData.at(i)[0], (outData.at(i)[0] / 1000));
        EXPECT_NEAR(expectedPoints.at(i).xyz[1], outData.at(i)[1], (outData.at(i)[1] / 1000));
        EXPECT_NEAR(expectedPoints.at(i).xyz[2], outData.at(i)[2], (outData.at(i)[2] / 1000));
    }
}