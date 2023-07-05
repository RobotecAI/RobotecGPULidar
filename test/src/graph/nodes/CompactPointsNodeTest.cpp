#include <PointsTestHelper.hpp>
#include <RaysTestHelper.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>

class CompactPointsNodeTest : public RGLTestWithParam<int>, public RGLRaysNodeTestHelper, public RGLTestUsePointsNodeHelper<TestPointIsHit>{
protected:
<<<<<<< HEAD
    rgl_node_t compactNode = nullptr;

    void preparePointsWithCompactGraph(int pointsCount, HitPointDensity hitPointDensity)
    {
        createTestUsePointsNode(pointsCount, identityTestTransform, hitPointDensity);
        // TODO(nebraszka): Test cases when the input struct has padding inside
        ASSERT_EQ(sizeof(TestPoint), sizeof(float) * 5);
=======
    std::vector<rgl_node_t> compactNodes;
    std::vector<rgl_node_t> pointsTransformNodes;

    CompactPointsNodeTest()
        : compactNodes(15, nullptr), pointsTransformNodes(15, nullptr) {}

    void prepareAndRunGraph(int pointsCount, HitPointDensity hitPointDensity)
    {
        prepareUsePointsNode(pointsCount, hitPointDensity);
>>>>>>> a7569a1 (Finished version with points generator)

        ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNodes.at(0)));
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
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    EXPECT_THAT(compactNodes.at(0), testing::NotNull());
}

TEST_F(CompactPointsNodeTest, valid_argument_node_is_not_nullptr)
{
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    ASSERT_THAT(compactNodes.at(0), testing::NotNull());

    // If (*compactNodes.at(0)) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
}

TEST_F(CompactPointsNodeTest, invalid_pipeline_when_no_input_node)
{
    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNodes.at(0)), "looked for IPointsNode");
}

TEST_F(CompactPointsNodeTest, invalid_pipeline_when_incorrect_input_node)
{
    createTestUseRaysNode(100);

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, compactNodes.at(0)));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(useRaysNode), "looked for IPointsNode");
}

TEST_P(CompactPointsNodeTest, should_remove_all_points_when_all_non_hit)
{
    int pointsCount = GetParam();
    prepareAndRunGraph(pointsCount, HitPointDensity::ALL_NON_HIT);

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, 0);

    // Check if the contents of outData have changed (they should not have)
    std::vector<::Field<XYZ_F32>::type> outData { { 2.0f, 3.0f, 5.0f }, { 7.0f, 11.0f, 13.0f }, { 17.0f, 19.0f, 23.0f } };
    auto outDataCopy = outData;

    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(compactNodes.at(0), RGL_FIELD_XYZ_F32, outData.data()));

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
    prepareAndRunGraph(pointsCount, HitPointDensity::ALL_HIT);

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount);
}

TEST_P(CompactPointsNodeTest, should_remove_as_many_points_as_non_hits)
{
    int pointsCount = GetParam();
    prepareAndRunGraph(pointsCount, HitPointDensity::RANDOM);

    // Print random seed for reproducibility
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount - randomNonHitCount);
}

TEST_P(CompactPointsNodeTest, should_not_change_hit_points)
{
    int pointsCount = GetParam();
<<<<<<< HEAD
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
=======

    prepareAndRunGraph(pointsCount, HitPointDensity::RANDOM);
    
    // Print random seed for reproducibility
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

    RGLTestPointsIsHitGenerator* pointsGenerator = dynamic_cast<RGLTestPointsIsHitGenerator*>(generator.get());
    std::vector<TestPointIsHit> expectedHitPoints = pointsGenerator->separateHitPoints();
    std::vector<TestPointIsHit> returnedPoints = getResults(compactNodes.at(0));
    verifyResults(returnedPoints, expectedHitPoints);
}

TEST_F(CompactPointsNodeTest, should_not_change_result_when_multiple_compactions_applied_to_same_data)
{
    int pointsCount = 1000;

    prepareUsePointsNode(pointsCount, HitPointDensity::RANDOM);
    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    auto results = getResults(usePointsNode);

    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

    for (int i = 0; i < compactNodes.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(i)));
    }

    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNodes.at(0)));
    for (int i = 1; i < compactNodes.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNodes.at(i - 1), compactNodes.at(i)));
    }

    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    RGLTestPointsIsHitGenerator* pointsGenerator = dynamic_cast<RGLTestPointsIsHitGenerator*>(generator.get());
    std::vector<TestPointIsHit> expectedHitPoints = pointsGenerator->separateHitPoints();
    std::vector<TestPointIsHit> returnedPoints = getResults(compactNodes.at(compactNodes.size() - 1));
    verifyResults(returnedPoints, expectedHitPoints);
}

TEST_F(CompactPointsNodeTest, should_not_change_result_when_multiple_compactions_applied_to_changing_data)
{
    int pointsCount = 1000;

    prepareUsePointsNode(pointsCount, HitPointDensity::RANDOM);
    // Print random seed for reproducibility
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

    const rgl_mat3x4f trans = Mat3x4f::translation(1.5f, 0.5f, 0.8f).toRGL();

    for (int i = 0; i < compactNodes.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(i)));
        ASSERT_RGL_SUCCESS(rgl_node_points_transform(&pointsTransformNodes.at(i), &trans));
    }

    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, pointsTransformNodes.at(0)));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(pointsTransformNodes.at(0), compactNodes.at(0)));

    for (int i = 1; i < compactNodes.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNodes.at(i - 1), pointsTransformNodes.at(i)));
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(pointsTransformNodes.at(i), compactNodes.at(i)));
    }

    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    RGLTestPointsIsHitGenerator* pointsGenerator = dynamic_cast<RGLTestPointsIsHitGenerator*>(generator.get());
    std::vector<TestPointIsHit> hitPoints = pointsGenerator->separateHitPoints();

    for (auto compactNode : compactNodes) {
        std::vector<TestPointIsHit> returnedPoints = getResults(compactNode);
        for(auto& point : hitPoints) {
            point.transform(Mat3x4f::fromRGL(trans));
        }
        verifyResults(returnedPoints, hitPoints);
    }
>>>>>>> a7569a1 (Finished version with points generator)
}

TEST_F(CompactPointsNodeTest, should_warn_when_empty_point_cloud)
{
    rgl_node_t emptyPointCloudOutputNode = simulateEmptyPointCloudOutputNode();

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(emptyPointCloudOutputNode, compactNodes.at(0)));

    //  TODO: Causes Segmentation fault
    //  ASSERT_RGL_SUCCESS(rgl_graph_run(compactNodes.at(0)));

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
    RGLTestUsePointsNodeHelper<TestPoint> helper;

    helper.prepareUsePointsNode(100);

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(helper.usePointsNode, compactNodes.at(0)));

    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNodes.at(0)), "IS_HIT_I32");
}

