#include <UsePointsNodeHelper.hpp>
#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>

class FromArrayPointsNodeTest : public RGLTestWithParam<int>, public RGLTestUsePointsNodeHelper<TestPointIsHit> {
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
    RGLTestPointsGenerator pointsGenerator;
    std::vector<rgl_field_t> fields = pointsGenerator.getPointFields();

    auto initializeArgumentsLambda = [this, &pointsGenerator]() {
        int pointsCount = GetParam();
        pointsGenerator.generateTestPoints(pointsCount);
        usePointsNode = nullptr;
    };

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, pointsGenerator.getPointsData(), pointsGenerator.getPointsSize(), fields.data(), 0), "field_count > 0");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, pointsGenerator.getPointsData(), pointsGenerator.getPointsSize(), nullptr, fields.size()), "fields != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, pointsGenerator.getPointsData(), 0, fields.data(), fields.size()), "points_count > 0");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, nullptr, pointsGenerator.getPointsSize(), fields.data(), fields.size()), "points != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(nullptr, pointsGenerator.getPointsData(), pointsGenerator.getPointsSize(), fields.data(), fields.size()), "node != nullptr");
}

TEST_P(FromArrayPointsNodeTest, valid_arguments)
{
    int pointsCount = GetParam();
    RGLTestPointsGenerator pointsGenerator;
    std::vector<rgl_field_t> fields = pointsGenerator.getPointFields();
    pointsGenerator.generateTestPoints(pointsCount);
    rgl_node_t usePointsNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, pointsGenerator.getPointsData(), pointsGenerator.getPointsSize(), fields.data(), fields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());
}

TEST_P(FromArrayPointsNodeTest, use_case)
{
    int pointsCount = GetParam();

    prepareUsePointsNode(pointsCount, HitPointDensity::HALF_HIT);
    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    auto expectedPoints = generator->getPointsVector();
    auto expectedSize = generator->getPointsSize();

    for (auto field : generator->getPointFields()) {
        int32_t outCount, outSizeOf;
        EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(usePointsNode, field, &outCount, &outSizeOf));
        EXPECT_EQ(outCount, expectedSize);
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
