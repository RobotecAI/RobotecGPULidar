#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <repr.hpp>
#include <utils.hpp>

class TransformPointsNodeTest : public RGLAutoCleanupTestWithParam<std::tuple<int, rgl_mat3x4f>>, public RGLGraphTest {
};

INSTANTIATE_TEST_SUITE_P(
    TransformPointsNodeTests, TransformPointsNodeTest,
    testing::Combine(
        testing::Values(1, 10, 100000),
        testing::Values(identityTestTransform, translationTestTransform, rotationTestTransform, scalingTestTransform, complexTestTransform)));

TEST_P(TransformPointsNodeTest, invalid_arguments)
{
    auto transform = std::get<1>(GetParam());
    rgl_node_t transformPointsNode = nullptr;

    auto initializeArgumentsLambda = [&transform, &transformPointsNode]() {
        transform = std::get<1>(GetParam());
        transformPointsNode = nullptr;
    };

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, nullptr), "node != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, &transform), "node != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(&transformPointsNode, nullptr), "transform != nullptr");
}

TEST_P(TransformPointsNodeTest, valid_arguments)
{
    auto transform = std::get<1>(GetParam());
    rgl_node_t transformPointsNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformPointsNode, &transform));
    ASSERT_THAT(transformPointsNode, testing::NotNull());

    // If (*node) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&transformPointsNode, &transform));
}

TEST_P(TransformPointsNodeTest, use_case)
{
    auto [pointsCount, transform] = GetParam();
    auto inPoints = GenerateTestPointsArray(pointsCount);

    rgl_node_t usePointsNode = nullptr;
    rgl_node_t transformNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());

    EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformNode, &transform));
    ASSERT_THAT(transformNode, testing::NotNull());

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, transformNode));

    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    int32_t outCount, outSizeOf;
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(transformNode, XYZ_F32, &outCount, &outSizeOf));

    EXPECT_EQ(outCount, inPoints.size());
    EXPECT_EQ(outSizeOf, getFieldSize(XYZ_F32));

    std::vector<::Field<XYZ_F32>::type> outData;
    outData.reserve(outCount);
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(transformNode, XYZ_F32, outData.data()));

    std::vector<TestPointStruct> expectedPoints = GenerateTestPointsArray(outCount, transform);

    // TODO When we are testing big values of point translation, numerical errors appears.
    //  For example for 100000 unit of translation, error after rotation can extend 0.001 unit.
    //  To investigate in better times.

    // TODO 2 In the future, this and other tests, should check if every possible field has proper value, even if intact.
    for (int i = 0; i < pointsCount; ++i) {
        // For now use percent (0.1%) of value as error treshold.
        EXPECT_NEAR(expectedPoints[i].xyz[0], outData[i][0], (outData[i][0]/ 1000));
        EXPECT_NEAR(expectedPoints[i].xyz[1], outData[i][1], (outData[i][1] / 1000));
        EXPECT_NEAR(expectedPoints[i].xyz[2], outData[i][2], (outData[i][2] / 1000));
    }
}