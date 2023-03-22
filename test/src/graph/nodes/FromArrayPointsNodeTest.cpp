#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

struct PointStruct
{
	::Field<XYZ_F32>::type xyz;
	::Field<IS_HIT_I32>::type isHit;
	::Field<INTENSITY_F32>::type intensity;
};

class FromArrayPointsNodeTest : public RGLAutoCleanupTestWithParam<int>
{
protected:
	static std::vector<PointStruct> GeneratePointsArray(int count)
	{
		std::vector<PointStruct> points;
		for (int i = 0; i < count; ++i) {
			points.emplace_back(PointStruct{
			    .xyz = {i, i + 1, i + 2},
                  .isHit = i % 2, .intensity = 100
            });
		}
		return points;
	}

	std::vector<rgl_field_t> pointFields = {XYZ_F32, IS_HIT_I32, INTENSITY_F32};
};

INSTANTIATE_TEST_SUITE_P(FromArrayPointsNodeTests, FromArrayPointsNodeTest, testing::Values(1, 10, 100000),
                         [](const auto& info) { return "pointsCount_" + std::to_string(info.param); });

TEST_P(FromArrayPointsNodeTest, invalid_arguments)
{
	int pointsCount = GetParam();
	auto inPoints = GeneratePointsArray(pointsCount);
	rgl_node_t usePointsNode = nullptr;

	auto initializeArgumentsLambda = [&pointsCount, &inPoints, &usePointsNode]() {
		pointsCount = FromArrayPointsNodeTest::GetParam();
		inPoints = FromArrayPointsNodeTest::GeneratePointsArray(pointsCount);
		usePointsNode = nullptr;
	};

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), 0), "field_count > 0");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), nullptr, pointFields.size()),
	    "fields != nullptr");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), 0, pointFields.data(), pointFields.size()),
	    "points_count > 0");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(
	    rgl_node_points_from_array(&usePointsNode, nullptr, inPoints.size(), pointFields.data(), pointFields.size()),
	    "points != nullptr");

	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(
	    rgl_node_points_from_array(nullptr, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()),
	    "node != nullptr");
}
TEST_P(FromArrayPointsNodeTest, valid_arguments)
{
	int pointsCount = GetParam();
	auto inPoints = GeneratePointsArray(pointsCount);

	rgl_node_t usePointsNode = nullptr;

	EXPECT_RGL_SUCCESS(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
	ASSERT_THAT(usePointsNode, testing::NotNull());
}

TEST_P(FromArrayPointsNodeTest, use_case)
{
	int pointsCount = GetParam();
	auto inPoints = GeneratePointsArray(pointsCount);

	rgl_node_t usePointsNode = nullptr;

	EXPECT_RGL_SUCCESS(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
	ASSERT_THAT(usePointsNode, testing::NotNull());

	EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	std::vector<PointStruct> expectedPoints = GeneratePointsArray(pointsCount);

	std::vector<::Field<XYZ_F32>::type> expectedXYZ;
	expectedXYZ.reserve(pointsCount);

	for (auto field : pointFields) {
		int32_t outCount, outSizeOf;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(usePointsNode, field, &outCount, &outSizeOf));
		EXPECT_EQ(outCount, inPoints.size());
		EXPECT_EQ(outSizeOf, getFieldSize(field));

		switch (field) {
			case XYZ_F32: {
				std::vector<::Field<XYZ_F32>::type> outData;
				outData.reserve(outCount);
				EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData.data()));
				for (int i = 0; i < outCount; ++i) {
					EXPECT_NEAR(expectedPoints[i].xyz[0], outData[i][0], EPSILON_F);
					EXPECT_NEAR(expectedPoints[i].xyz[1], outData[i][1], EPSILON_F);
					EXPECT_NEAR(expectedPoints[i].xyz[2], outData[i][2], EPSILON_F);
				}
				break;
			}
			case IS_HIT_I32: {
				std::vector<::Field<IS_HIT_I32>::type> outData;
				outData.reserve(outCount);
				EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData.data()));
				for (int i = 0; i < outCount; ++i) {
					EXPECT_NEAR(expectedPoints[i].isHit, outData[i], EPSILON_F);
				}
				break;
			}
			case INTENSITY_F32: {
				std::vector<::Field<INTENSITY_F32>::type> outData;
				outData.reserve(outCount);
				EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData.data()));
				for (int i = 0; i < outCount; ++i) {
					EXPECT_NEAR(expectedPoints[i].intensity, outData[i], EPSILON_F);
				}
				break;
			}
		}
	}
}