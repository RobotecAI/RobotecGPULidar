#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <PointsTestHelper.hpp>

class CompactPointsNodeTest : public RGLTestWithParam<int>, public RGLPointsTestHelper{
protected:
    rgl_node_t compactNode;

    CompactPointsNodeTest()
    {
        compactNode = nullptr;
    }
};

INSTANTIATE_TEST_SUITE_P(
    CompactPointsNodeTests, CompactPointsNodeTest,
    testing::Values(1, 100, maxGPUCoresTestCount),
    [](const auto& info) {
        return "pointsCount_" + std::to_string(info.param);
    });

TEST_F(CompactPointsNodeTest, invalid_arguments)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "node != nullptr");
}

TEST_F(CompactPointsNodeTest, valid_argument_node_is_nullptr)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    EXPECT_THAT(compactNode, testing::NotNull());
}

TEST_F(CompactPointsNodeTest, valid_argument_node_is_not_nullptr)
{
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_THAT(compactNode, testing::NotNull());

    // If (*compactNode) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
}

TEST_P(CompactPointsNodeTest, should_remove_all_points_when_all_non_hit)
{
    int pointsCount = GetParam();

    CreateTestUsePointsNode(pointsCount, identityTestTransform, HitPointDensity::ALL_NON_HIT);
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, 0);
}

TEST_P(CompactPointsNodeTest, should_not_remove_any_point_when_all_hit)
{
    int pointsCount = GetParam();

    CreateTestUsePointsNode(pointsCount, identityTestTransform, HitPointDensity::ALL_HIT);
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount);
}

TEST_P(CompactPointsNodeTest, should_remove_as_many_points_as_non_hit)
{
    int pointsCount = GetParam();

    CreateTestUsePointsNode(pointsCount, identityTestTransform, HitPointDensity::RANDOM);
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount - randomNonHitCount);
}

TEST_P(CompactPointsNodeTest, should_not_remove_hit_points)
{
    int pointsCount = GetParam();

    CreateTestUsePointsNode(pointsCount, identityTestTransform, HitPointDensity::RANDOM);
    std::vector<TestPointStruct> expectedHitPoints = separateHitPoints();
    if(expectedHitPoints.empty())
        return;

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    for (auto field : pointFields) {
        int32_t outCount, outSizeOf;
        EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, field, &outCount, &outSizeOf));
        EXPECT_EQ(outCount, expectedHitPoints.size());
        EXPECT_EQ(outSizeOf, getFieldSize(field));

        switch(field) {
            case XYZ_F32: {
                std::vector<::Field<XYZ_F32>::type> outData;
                outData.resize(outCount);
                EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, field, outData.data()));
                for (int i = 0; i < outCount; ++i) {
                    EXPECT_NEAR(expectedHitPoints.at(i).xyz[0], outData.at(i)[0], EPSILON_F);
                    EXPECT_NEAR(expectedHitPoints.at(i).xyz[1], outData.at(i)[1], EPSILON_F);
                    EXPECT_NEAR(expectedHitPoints.at(i).xyz[2], outData.at(i)[2], EPSILON_F);
                }
                break;
            }
            case IS_HIT_I32: {
                std::vector<::Field<IS_HIT_I32>::type> outData;
                outData.resize(outCount);
                EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, field, outData.data()));
                for (int i = 0; i < outCount; ++i) {
                    EXPECT_EQ(1, outData.at(i));
                }
                break;
            }
            case INTENSITY_F32: {
                std::vector<::Field<INTENSITY_F32>::type> outData;
                outData.resize(outCount);
                EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, field, outData.data()));
                for (int i = 0; i < outCount; ++i) {
                    EXPECT_NEAR(expectedHitPoints.at(i).intensity, outData.at(i), EPSILON_F);
                }
                break;
            }
        }
    }
}
