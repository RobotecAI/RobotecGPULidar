#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <PointsTestHelper.hpp>
#include <RaysTestHelper.hpp>

class CompactPointsNodeTest : public RGLTestWithParam<int>, public RGLPointsNodeTestHelper, public RGLRaysNodeTestHelper{
protected:
    rgl_node_t compactNode = nullptr;

    void preparePointsWithCompactGraph(int pointsCount, HitPointDensity hitPointDensity)
    {
        createTestUsePointsNode(pointsCount, identityTestTransform, hitPointDensity);
        // TODO(nebraszka): Test cases when the input struct has padding inside
        ASSERT_EQ(sizeof(TestPoint), sizeof(float) * 5);

        ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));
        ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));
    }
};

INSTANTIATE_TEST_SUITE_P(
    CompactPointsNodeTests, CompactPointsNodeTest,
    testing::Values(1, 100, maxGPUCoresTestCount),
    [](const auto& info) {
        return "pointsCount_" + std::to_string(info.param);
    });

TEST_F(CompactPointsNodeTest, invalid_argument_node)
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

TEST_F(CompactPointsNodeTest, invalid_pipeline_when_no_input_node)
{
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNode), "looked for IPointsNode");
}

TEST_F(CompactPointsNodeTest, invalid_pipeline_when_incorrect_input_node)
{
    createTestUseRaysNode(100);

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, compactNode));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(useRaysNode), "looked for IPointsNode");
}

TEST_P(CompactPointsNodeTest, should_remove_all_points_when_all_non_hit)
{
    int pointsCount = GetParam();
    preparePointsWithCompactGraph(pointsCount, HitPointDensity::ALL_NON_HIT);

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, 0);

    std::vector<::Field<XYZ_F32>::type> outData{ {2.0f, 3.0f, 5.0f}, {7.0f, 11.0f, 13.0f}, {17.0f, 19.0f, 23.0f} };
    auto outDataCopy = outData;

    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, RGL_FIELD_XYZ_F32, outData.data()));

    // Check if the contents of outData have changed (they should not have)
    ASSERT_EQ(outData.size(), outDataCopy.size());
    for (int i = 0; i < outData.size(); ++i) {
        EXPECT_EQ(outData.at(i)[0], outDataCopy.at(i)[0]);
        EXPECT_EQ(outData.at(i)[1], outDataCopy.at(i)[1]);
        EXPECT_EQ(outData.at(i)[2], outDataCopy.at(i)[2]);
    }
}

TEST_P(CompactPointsNodeTest, should_not_remove_any_point_when_all_hit)
{
    int pointsCount = GetParam();
    preparePointsWithCompactGraph(pointsCount, HitPointDensity::ALL_HIT);

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount);
}

TEST_P(CompactPointsNodeTest, should_remove_as_many_points_as_non_hits)
{
    int pointsCount = GetParam();
    preparePointsWithCompactGraph(pointsCount, HitPointDensity::RANDOM);

    // Print random seed for reproducibility
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount - randomNonHitCount);
}

TEST_P(CompactPointsNodeTest, should_not_change_hit_points)
{
    int pointsCount = GetParam();
    preparePointsWithCompactGraph(pointsCount, HitPointDensity::RANDOM);

    // Print random seed for reproducibility
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

    std::vector<TestPoint> expectedHitPoints = separateHitPoints();

    for (auto field : pointFields) {
        int32_t outCount, outSizeOf;
        EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(compactNode, field, &outCount, &outSizeOf));
        EXPECT_EQ(outCount, expectedHitPoints.size());
        EXPECT_EQ(outSizeOf, getFieldSize(field));

        switch (field) {
        case XYZ_F32: {
            std::vector<::Field<XYZ_F32>::type> outData;
            outData.resize(outCount);
            if(outCount == 0) {
                EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(compactNode, field, outData.data()), "data != nullptr");
            }
            else {
                EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, field, outData.data()));
                for (int i = 0; i < outCount; ++i) {
                    EXPECT_NEAR(expectedHitPoints.at(i).xyz[0], outData.at(i)[0], EPSILON_F);
                    EXPECT_NEAR(expectedHitPoints.at(i).xyz[1], outData.at(i)[1], EPSILON_F);
                    EXPECT_NEAR(expectedHitPoints.at(i).xyz[2], outData.at(i)[2], EPSILON_F);
                }
            }
            break;
        }
        case IS_HIT_I32: {
            std::vector<::Field<IS_HIT_I32>::type> outData;
            outData.resize(outCount);
            if(outCount == 0) {
                EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(compactNode, field, outData.data()), "data != nullptr");
            }
            else {
                EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, field, outData.data()));
                for (int i = 0; i < outCount; ++i) {
                    EXPECT_EQ(1, outData.at(i));
                }
            }
            break;
        }
        case INTENSITY_F32: {
            std::vector<::Field<INTENSITY_F32>::type> outData;
            outData.resize(outCount);
            if(outCount == 0) {
                EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(compactNode, field, outData.data()), "data != nullptr");
            }
            else {
                EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compactNode, field, outData.data()));
                for (int i = 0; i < outCount; ++i) {
                    EXPECT_EQ(expectedHitPoints.at(i).intensity, outData.at(i));
                }
            }
            break;
        }
        }
    }
}

TEST_F(CompactPointsNodeTest, should_warn_when_empty_point_cloud)
{
    rgl_node_t emptyPointCloudInputNode = simulateEmptyPointCloudInputNode();

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(emptyPointCloudInputNode, compactNode));

    //  TODO: Causes Segmentation fault
    //  ASSERT_RGL_SUCCESS(rgl_graph_run(compactNode));

    FAIL();
}

TEST_F(CompactPointsNodeTest, same_data)
{
    //TODO(nebraszka) Still needs to be implemented
}

TEST_F(CompactPointsNodeTest, changing_data)
{
    //TODO(nebraszka) Still needs to be implemented
}


TEST_F(CompactPointsNodeTest, without_IS_HIT_field)
{
    std::vector<TestPointWithoutIsHit> points = generateTestPointsWithoutIsHit(100);

    ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), pointFieldsWithoutIsHit.data(), pointFieldsWithoutIsHit.size()));
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));

    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNode), "IS_HIT_I32");
}

