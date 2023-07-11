#include <UsePointsNodeHelper.hpp>
#include <RaysNodeHelper.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>

constexpr size_t MULTIPLE_COMPACTIONS_COUNT = 15;
constexpr size_t RAYS_COUNT = 15;
constexpr auto TEST_SEED_ENV_VAR = "RGL_COMPACT_TEST_SEED";

class CompactPointsNodeTest : public RGLTestWithParam<int>, public RGLTestRaysNodeHelper, public RGLTestUsePointsNodeHelper<TestPointIsHit>{
protected:
    std::vector<rgl_node_t> compactNodes;
    std::vector<rgl_node_t> pointsTransformNodes;

    CompactPointsNodeTest()
        : compactNodes(MULTIPLE_COMPACTIONS_COUNT, nullptr), pointsTransformNodes(MULTIPLE_COMPACTIONS_COUNT, nullptr) 
    {
        char* envSeed = std::getenv(TEST_SEED_ENV_VAR);
        if (envSeed != nullptr) {
            unsigned seed = std::stoul(envSeed);
            setSeed(seed);
        }
    }

    void prepareAndRunGraph(int pointsCount, HitPointDensity hitPointDensity)
    {
        prepareUsePointsNode(pointsCount, hitPointDensity);

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
    createTestUseRaysNode(RAYS_COUNT);

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
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", generator->getRandomSeed());

    int32_t hitpointCount, pointSize;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
    EXPECT_EQ(hitpointCount, pointsCount - randomNonHitCount);
}

TEST_P(CompactPointsNodeTest, should_not_change_hit_points)
{
    int pointsCount = GetParam();

    prepareAndRunGraph(pointsCount, HitPointDensity::RANDOM);
    
    // Print random seed for reproducibility
    fmt::print(stderr, "Compact Points Node Test random seed: {}\n", generator->getRandomSeed());

    RGLTestPointsIsHitGenerator* pointsGenerator = dynamic_cast<RGLTestPointsIsHitGenerator*>(generator.get());
    std::vector<TestPointIsHit> expectedHitPoints = pointsGenerator->separateHitPoints();
    std::vector<TestPointIsHit> returnedPoints = getResults(compactNodes.at(0));
    verifyResults(returnedPoints, expectedHitPoints);
}

TEST_P(CompactPointsNodeTest, should_not_change_result_when_multiple_compactions_applied_to_same_data)
{
    int pointsCount = GetParam();

    // TODO: In the future it should be changed to HitPointDensity::RANDOM, 
    // currently compact on empty point cloud causes segfault, hence the decision to set all points as hit,
    // otherwise it would affect the result of this test
    prepareUsePointsNode(pointsCount, HitPointDensity::ALL_HIT);

    ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    auto results = getResults(usePointsNode);

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

TEST_P(CompactPointsNodeTest, should_not_change_result_when_multiple_compactions_applied_to_changing_data)
{
    int pointsCount = GetParam();

    // TODO: In the future it should be changed to HitPointDensity::RANDOM, 
    // currently compact on empty point cloud causes segfault, hence the decision to set all points as hit,
    // otherwise it would affect the result of this test
    prepareUsePointsNode(pointsCount, HitPointDensity::ALL_HIT);

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

TEST_F(CompactPointsNodeTest, without_IS_HIT_field)
{
    RGLTestUsePointsNodeHelper<TestPoint> helper;

    helper.prepareUsePointsNode(100);

    ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(helper.usePointsNode, compactNodes.at(0)));

    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNodes.at(0)), "IS_HIT_I32");
}

