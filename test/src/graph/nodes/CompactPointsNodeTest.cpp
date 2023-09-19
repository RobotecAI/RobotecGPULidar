#include <CompactPointsNodeHelper.hpp>
#include <RaysNodeHelper.hpp>

class CompactPointsNodeTest : public RGLTestWithParam<int>, public RGLTestRaysNodeHelper, public CompactPointsNodeHelper
{};

INSTANTIATE_TEST_SUITE_P(CompactPointsNodeTests, CompactPointsNodeTest, testing::Values(1, 100, maxGPUCoresTestCount),
                         [](const auto& info) { return "pointsCount_" + std::to_string(info.param); });

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
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNodes.at(0)), "looked for IPointsNode");
}

TEST_P(CompactPointsNodeTest, points_all_non_hit)
{
	int pointsCount = GetParam();
	generatePoints(pointsCount, HitPointDensity::ALL_NON_HIT);
	prepareAndRunUsePointsCompactGraph();

	int32_t hitpointCount, pointSize;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
	EXPECT_EQ(hitpointCount, 0);

	// Check if the contents of outData have changed (they should not have)
	std::vector<::Field<XYZ_F32>::type> outData{
	    { 2.0f,  3.0f,  5.0f},
        { 7.0f, 11.0f, 13.0f},
        {17.0f, 19.0f, 23.0f}
    };
	auto outDataCopy = outData;

	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(compactNodes.at(0), RGL_FIELD_XYZ_F32, outData.data()));
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
	generatePoints(pointsCount, HitPointDensity::ALL_HIT);
	auto inPointsCopy = inPoints;

	prepareAndRunUsePointsCompactGraph();

	int32_t hitPointCount, pointSize;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitPointCount, &pointSize));
	ASSERT_EQ(hitPointCount, pointsCount);

	getResults(usePointsNode);
	for (int i = 0; i < pointsCount; ++i) {
		EXPECT_EQ(outPoints.at(i), inPointsCopy.at(i));
	}
}

TEST_P(CompactPointsNodeTest, points_random_hit)
{
	int pointsCount = GetParam();
	generatePoints(pointsCount, HitPointDensity::RANDOM);
	auto inPointsCopy = inPoints;

	prepareAndRunUsePointsCompactGraph();

	// Remove non hit points from vector
	inPointsCopy.erase(
	    std::remove_if(inPointsCopy.begin(), inPointsCopy.end(), [](const auto& point) { return point.isHit == 0; }),
	    inPointsCopy.end());

	// Print random seed for reproducibility
	// TODO(nebraszka) print seed in xml report, not in the console
	fmt::print(stderr, "Compact Points Node Test random seed: {}\n", randomSeed);

	int32_t hitpointCount, pointSize;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(compactNodes.at(0), RGL_FIELD_XYZ_F32, &hitpointCount, &pointSize));
	// points.size() może być mylące
	ASSERT_EQ(hitpointCount, inPointsCopy.size());

	// Hit points should not be changed
	getResults(compactNodes.at(0));
	for (int i = 0; i < outPoints.size(); ++i) {
		EXPECT_EQ(outPoints.at(i).isHit, inPointsCopy.at(i).isHit);
	}
}

TEST_P(CompactPointsNodeTest, multiple_compactions_applied_to_same_data)
{
	int pointsCount = GetParam();
	generatePoints(pointsCount, HitPointDensity::ALL_HIT);
	auto inPointsCopy = inPoints;

	for (auto& compactNode : compactNodes) {
		ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
		ASSERT_THAT(compactNode, testing::NotNull());
	}

	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(),
	                                              IsHitPoint::getPointFields().data(), IsHitPoint::getPointFields().size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNodes.at(0)));
	for (int i = 1; i < compactNodes.size(); ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNodes.at(i - 1), compactNodes.at(i)));
	}
	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	getResults(compactNodes.at(compactNodes.size() - 1));

	//Separate hit points
	inPointsCopy.erase(
	    std::remove_if(inPointsCopy.begin(), inPointsCopy.end(), [](const auto& point) { return point.isHit == 0; }),
	    inPointsCopy.end());

	for (int i = 0; i < outPoints.size(); ++i) {
		EXPECT_EQ(outPoints.at(i).isHit, inPointsCopy.at(i).isHit);
	}
}

TEST_P(CompactPointsNodeTest, multiple_compactions_applied_to_changing_data)
{
	int pointsCount = GetParam();
	generatePoints(pointsCount, HitPointDensity::ALL_HIT);
	auto inPointsCopy = inPoints;

	const rgl_mat3x4f trans = Mat3x4f::translation(1.5f, 0.5f, 0.8f).toRGL();

	for (int i = 0; i < compactNodes.size(); ++i) {
		ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(i)));
		ASSERT_RGL_SUCCESS(rgl_node_points_transform(&pointsTransformNodes.at(i), &trans));
	}
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(),
	                                              IsHitPoint::getPointFields().data(), IsHitPoint::getPointFields().size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, pointsTransformNodes.at(0)));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(pointsTransformNodes.at(0), compactNodes.at(0)));
	for (int i = 1; i < compactNodes.size(); ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNodes.at(i - 1), pointsTransformNodes.at(i)));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(pointsTransformNodes.at(i), compactNodes.at(i)));
	}
	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	//Separate hit points
	inPointsCopy.erase(
	    std::remove_if(inPointsCopy.begin(), inPointsCopy.end(), [](const auto& point) { return point.isHit == 0; }),
	    inPointsCopy.end());

	for (auto compactNode : compactNodes) {
		getResults(compactNode);
		for (auto& point : inPointsCopy) {
			point.xyz = Mat3x4f::fromRGL(trans) * point.xyz;
		}
		for (int i = 0; i < outPoints.size(); ++i) {
			EXPECT_EQ(outPoints.at(i).isHit, inPointsCopy.at(i).isHit);
		}
	}
}

TEST_F(CompactPointsNodeTest, empty_point_cloud)
{
	//	generatePoints(1, HitPointDensity::ALL_NON_HIT);
	//	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
	//	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(),
	//	                                              IsHitPoint::getPointFields().data(), IsHitPoint::getPointFields().size()));
	//	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNodes.at(0)));
	//	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(1)));
	//	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNodes.at(0), compactNodes.at(1)));
	//
	//	//  TODO: Causes Segmentation fault
	//	ASSERT_RGL_SUCCESS(rgl_graph_run(compactNodes.at(0)));

	FAIL();
}

TEST_F(CompactPointsNodeTest, without_IS_HIT_field)
{
	int pointCount = 10;
	std::vector<Point> pointsWithoutIsHit(pointCount);

	for (int i = 0; i < pointCount; ++i) {
		auto currentPoint = Point{
		    .xyz = {i, i + 1, i + 2},
		};
		pointsWithoutIsHit.push_back(currentPoint);
	}

	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, pointsWithoutIsHit.data(), pointsWithoutIsHit.size(),
	                                              Point::getPointFields().data(), Point::getPointFields().size()));
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compactNodes.at(0)));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNodes.at(0)));

	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(compactNodes.at(0)), "IS_HIT_I32");
}
