#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <PointsTestHelper.hpp>

class FromArrayPointsNodeTest : public RGLTestWithParam<int>, public RGLPointsNodeTestHelper {
protected:
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
        inPoints = generateTestPointsArray(pointsCount);
        usePointsNode = nullptr;
    };

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), 0), "field_count > 0");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), nullptr, pointFields.size()), "fields != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, inPoints.data(), 0, pointFields.data(), pointFields.size()), "points_count > 0");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, nullptr, inPoints.size(), pointFields.data(), pointFields.size()), "points != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(nullptr, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()), "node != nullptr");
}
TEST_P(FromArrayPointsNodeTest, valid_arguments)
{
    int pointsCount = GetParam();
    auto inPoints = generateTestPointsArray(pointsCount);
    rgl_node_t usePointsNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());
}

TEST_P(FromArrayPointsNodeTest, use_case)
{
    int pointsCount = GetParam();

    createTestUsePointsNode(pointsCount);
    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    std::vector<TestPointStruct> expectedPoints = inPoints;

    for (auto field : pointFields) {
        int32_t outCount, outSizeOf;
        EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(usePointsNode, field, &outCount, &outSizeOf));
        EXPECT_EQ(outCount, inPoints.size());
        EXPECT_EQ(outSizeOf, getFieldSize(field));

        switch (field) {
        case XYZ_F32: {
            std::vector<::Field<XYZ_F32>::type> outData;
            outData.resize(outCount);
            EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData.data()));
            for (int i = 0; i < outCount; ++i) {
                EXPECT_NEAR(expectedPoints.at(i).xyz[0], outData.at(i)[0], EPSILON_F);
                EXPECT_NEAR(expectedPoints.at(i).xyz[1], outData.at(i)[1], EPSILON_F);
                EXPECT_NEAR(expectedPoints.at(i).xyz[2], outData.at(i)[2], EPSILON_F);
            }
            break;
        }
        case IS_HIT_I32: {
            std::vector<::Field<IS_HIT_I32>::type> outData;
            outData.resize(outCount);
            EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData.data()));
            for (int i = 0; i < outCount; ++i) {
                EXPECT_NEAR(expectedPoints.at(i).isHit, outData.at(i), EPSILON_F);
            }
            break;
        }
        case INTENSITY_F32: {
            std::vector<::Field<INTENSITY_F32>::type> outData;
            outData.resize(outCount);
            EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData.data()));
            for (int i = 0; i < outCount; ++i) {
                EXPECT_NEAR(expectedPoints.at(i).intensity, outData.at(i), EPSILON_F);
            }
            break;
        }
        }
    }
}
