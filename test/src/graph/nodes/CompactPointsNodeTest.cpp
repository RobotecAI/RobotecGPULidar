#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>

#include <api/apiCommon.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>
#include <ranges>

static constexpr int MULTIPLE_COMPACTIONS_COUNT = 15;

class CompactPointsNodeTest : public RGLTestWithParam<int>
{
public:
	static void SetUpTestCase()
	{
		static std::once_flag flag;
		std::call_once(flag, []() { fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed); });
	}

protected:
	rgl_node_t compactPointsNode = nullptr;

	std::vector<rgl_field_t> pointFields = {XYZ_VEC3_F32, IS_HIT_I32};

	std::pair<std::unique_ptr<TestPointCloud>, rgl_node_t> createPointCloud(
	    int pointsCount, const std::function<Field<IS_HIT_I32>::type(int)>& genIsHit)
	{
		std::unique_ptr<TestPointCloud> pointCloud = std::make_unique<TestPointCloud>(pointFields, pointsCount);
		auto pointIsHitValues = generateFieldValues(pointsCount, genIsHit);
		pointCloud->setFieldValues<IS_HIT_I32>(pointIsHitValues);
		return {std::move(pointCloud), pointCloud->createUsePointsNode()};
	}

	void runGraphWithAssertions(rgl_node_t& inNode)
	{
		ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
		ASSERT_THAT(compactPointsNode, testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, compactPointsNode));
		ASSERT_RGL_SUCCESS(rgl_graph_run(inNode));
	}
};

INSTANTIATE_TEST_SUITE_P(CompactPointsNodeTests, CompactPointsNodeTest, testing::Values(1, 100, maxGPUCoresTestCount),
                         [](const auto& info) { return "pointsCount_" + std::to_string(info.param); });

TEST_F(CompactPointsNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "node != nullptr");
}

TEST_F(CompactPointsNodeTest, valid_argument_node_is_nullptr)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	EXPECT_THAT(compactPointsNode, testing::NotNull());
}

TEST_F(CompactPointsNodeTest, valid_argument_node_is_not_nullptr)
{
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	ASSERT_THAT(compactPointsNode, testing::NotNull());

	// If (*compactPointsNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
}

TEST_F(CompactPointsNodeTest, invalid_pipeline_when_no_input_node)
{
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactPointsNode), "looked for IPointsNode");
}

TEST_F(CompactPointsNodeTest, invalid_pipeline_when_incorrect_input_node)
{
	rgl_node_t useRaysNode = nullptr;
	std::vector<rgl_mat3x4f> rayTf;
	rayTf.emplace_back(Mat3x4f::identity().toRGL());
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));

	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, compactPointsNode));
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactPointsNode), "looked for IPointsNode");
}

TEST_P(CompactPointsNodeTest, points_all_non_hit)
{
	int pointsCount = GetParam();

	auto&& [inPointCloud, inNode] = createPointCloud(pointsCount, genAllNonHit);

	runGraphWithAssertions(inNode);

	auto&& outPointCloud = std::make_unique<TestPointCloud>(TestPointCloud::createFromNode(compactPointsNode, pointFields));
	EXPECT_EQ(outPointCloud->getPointCount(), 0);

	// Check if the contents of outData have changed (they should not have)
	std::vector<::Field<XYZ_VEC3_F32>::type> outData{
	    { 2.0f,  3.0f,  5.0f},
        { 7.0f, 11.0f, 13.0f},
        {17.0f, 19.0f, 23.0f}
    };
	auto outDataCopy = outData;

	int32_t outCount, outSize;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactPointsNode, RGL_FIELD_XYZ_VEC3_F32, &outCount, &outSize));
	ASSERT_EQ(outSize, sizeof(Field<XYZ_VEC3_F32>::type));
	ASSERT_EQ(outCount, 0);

	// Verify that the data is unchanged; this is a sanity check to make sure that the data is not overwritten
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(compactPointsNode, RGL_FIELD_XYZ_VEC3_F32, outData.data()));
	ASSERT_EQ(outData.size(), outDataCopy.size());
	for (int i = 0; i < outData.size(); ++i) {
		EXPECT_EQ(outData.at(i)[0], outDataCopy.at(i)[0]);
		EXPECT_EQ(outData.at(i)[1], outDataCopy.at(i)[1]);
		EXPECT_EQ(outData.at(i)[2], outDataCopy.at(i)[2]);
	}
}

TEST_P(CompactPointsNodeTest, points_all_hit)
{
	int pointsCount = GetParam();

	auto&& [inPointCloud, inNode] = createPointCloud(pointsCount, genAllHit);

	runGraphWithAssertions(inNode);

	auto&& outPointCloud = std::make_unique<TestPointCloud>(TestPointCloud::createFromNode(compactPointsNode, pointFields));

	EXPECT_EQ(*outPointCloud, *inPointCloud);
}

TEST_P(CompactPointsNodeTest, points_random_hit)
{
	int pointsCount = GetParam();

	auto&& [inPointCloud, inNode] = createPointCloud(pointsCount, genRandHit);

	runGraphWithAssertions(inNode);

	auto&& inPointCloudWithoutNonHits = inPointCloud;
	inPointCloudWithoutNonHits->removeNonHitPoints();

	auto&& outPointCloud = std::make_unique<TestPointCloud>(TestPointCloud::createFromNode(compactPointsNode, pointFields));

	EXPECT_EQ(*outPointCloud, *inPointCloudWithoutNonHits);
}

TEST_P(CompactPointsNodeTest, multiple_compactions_applied_to_same_data)
{
	int pointsCount = GetParam();

	auto&& [inPointCloud, inNode] = createPointCloud(pointsCount, genRandHit);

	std::vector<rgl_node_t> compactNodes(MULTIPLE_COMPACTIONS_COUNT);
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
	ASSERT_THAT(compactNodes.at(0), testing::NotNull());

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, compactNodes.at(0)));
	for (int i = 1; i < MULTIPLE_COMPACTIONS_COUNT; ++i) {
		ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(i)));
		ASSERT_THAT(compactNodes.at(i), testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNodes.at(i - 1), compactNodes.at(i)));
	}
	ASSERT_RGL_SUCCESS(rgl_graph_run(inNode));

	auto&& inPointCloudWithoutNonHits = inPointCloud;
	inPointCloudWithoutNonHits->removeNonHitPoints();

	auto&& outPointCloud = std::make_unique<TestPointCloud>(
	    TestPointCloud::createFromNode(compactNodes.at(MULTIPLE_COMPACTIONS_COUNT - 1), pointFields));

	EXPECT_EQ(*outPointCloud, *inPointCloudWithoutNonHits);
}

TEST_F(CompactPointsNodeTest, should_warn_when_empty_point_cloud)
{
	rgl_node_t emptyPointCloudOutputNode = nullptr;
	createOrUpdateNode<EmptyNode>(&emptyPointCloudOutputNode);

	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(emptyPointCloudOutputNode, compactPointsNode));

	ASSERT_RGL_SUCCESS(rgl_graph_run(compactPointsNode));
}

TEST_F(CompactPointsNodeTest, without_IS_HIT_field)
{
	int pointCount = 10;
	std::vector<rgl_field_t> pointFields = {XYZ_VEC3_F32};

	auto&& inPointCloud = std::make_unique<TestPointCloud>(pointFields, pointCount);
	auto pointCoordValues = generateFieldValues(pointCount, genCoord);

	rgl_node_t usePointsNode = inPointCloud->createUsePointsNode();
	ASSERT_THAT(usePointsNode, testing::NotNull());
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactPointsNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactPointsNode));

	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactPointsNode), "IS_HIT_I32");
}
