#include <points.hpp>

class PointCloudTest : public ::testing::TestWithParam<int>
{
protected:
	std::vector<rgl_field_t> allNotDummyFields = {XYZ_F32,     IS_HIT_I32,  RAY_IDX_U32,  ENTITY_ID_I32,  INTENSITY_F32,
	                                              RING_ID_U16, AZIMUTH_F32, DISTANCE_F32, RETURN_TYPE_U8, TIME_STAMP_F64};

	std::vector<rgl_field_t> fieldsWithPaddings = {PADDING_32, XYZ_F32,       PADDING_8, IS_HIT_I32,
	                                               PADDING_16, INTENSITY_F32, PADDING_32};

	std::vector<rgl_field_t> fields = {XYZ_F32, IS_HIT_I32, INTENSITY_F32};

	struct TestPointStruct
	{
		Field<XYZ_F32>::type xyz;
		Field<IS_HIT_I32>::type isHit;
		Field<INTENSITY_F32>::type intensity;
	};

	std::vector<TestPointStruct> generateTestPointsArray(int count)
	{
		std::vector<TestPointStruct> points;
		points.reserve(count);
		for (int i = 0; i < count; ++i) {
			auto currentPoint = TestPointStruct{
			    .xyz = {i, i + 1, i + 2},
                  .isHit = i % 2, .intensity = 100
            };
			points.push_back(currentPoint);
		}
		return points;
	}
};

INSTANTIATE_TEST_SUITE_P(PointCloudTests, PointCloudTest, testing::Values(1, 100, maxGPUCoresTestCount),
                         [](const auto& info) { return "pointsCount_" + std::to_string(info.param); });

TEST_P(PointCloudTest, setter_and_getter_with_transform)
{
	int pointsCount = GetParam();

	PointCloud pointCloud = PointCloud(allNotDummyFields, pointsCount);
	EXPECT_EQ(pointCloud.getPointCount(), pointsCount);

	std::vector<Field<XYZ_F32>::type> pointCoord = generate(pointsCount, genCoord);
	std::vector<Field<IS_HIT_I32>::type> isHit = generate(pointsCount, genHalfHit);
	std::vector<Field<RAY_IDX_U32>::type> rayIdx = generate(pointsCount, genRayIdx);
	std::vector<Field<ENTITY_ID_I32>::type> entityId = generate(pointsCount, genEntityId);
	std::vector<Field<INTENSITY_F32>::type> intensity = generate(pointsCount, genIntensity);
	std::vector<Field<RING_ID_U16>::type> ringId = generate(pointsCount, genRingId);
	std::vector<Field<AZIMUTH_F32>::type> azimuth = generate(pointsCount, genAzimuth);
	std::vector<Field<DISTANCE_F32>::type> distances = generate(pointsCount, genDistance);
	std::vector<Field<RETURN_TYPE_U8>::type> returnType = generate(pointsCount, genReturnType);
	std::vector<Field<TIME_STAMP_F64>::type> timeStamp = generate(pointsCount, genTimeStamp);
	pointCloud.setFieldValues<XYZ_F32>(pointCoord);
	pointCloud.setFieldValues<IS_HIT_I32>(isHit);
	pointCloud.setFieldValues<RAY_IDX_U32>(rayIdx);
	pointCloud.setFieldValues<ENTITY_ID_I32>(entityId);
	pointCloud.setFieldValues<INTENSITY_F32>(intensity);
	pointCloud.setFieldValues<RING_ID_U16>(ringId);
	pointCloud.setFieldValues<AZIMUTH_F32>(azimuth);
	pointCloud.setFieldValues<DISTANCE_F32>(distances);
	pointCloud.setFieldValues<RETURN_TYPE_U8>(returnType);
	pointCloud.setFieldValues<TIME_STAMP_F64>(timeStamp);

	pointCloud.transform(Mat3x4f::fromRGL(complexTestTransform));

	std::vector<Field<XYZ_F32>::type> outPointsCoord = pointCloud.getFieldValues<XYZ_F32>();
	for (int i = 0; i < pointsCount; i++) {
		pointCoord.at(i) = Mat3x4f::fromRGL(complexTestTransform) * pointCoord.at(i);
		EXPECT_EQ(outPointsCoord.at(i)[0], pointCoord.at(i)[0]);
		EXPECT_EQ(outPointsCoord.at(i)[1], pointCoord.at(i)[1]);
		EXPECT_EQ(outPointsCoord.at(i)[2], pointCoord.at(i)[2]);
	}
	EXPECT_EQ(pointCloud.getFieldValues<IS_HIT_I32>(), isHit);
	EXPECT_EQ(pointCloud.getFieldValues<RAY_IDX_U32>(), rayIdx);
	EXPECT_EQ(pointCloud.getFieldValues<ENTITY_ID_I32>(), entityId);
	EXPECT_EQ(pointCloud.getFieldValues<INTENSITY_F32>(), intensity);
	EXPECT_EQ(pointCloud.getFieldValues<RING_ID_U16>(), ringId);
	EXPECT_EQ(pointCloud.getFieldValues<AZIMUTH_F32>(), azimuth);
	EXPECT_EQ(pointCloud.getFieldValues<DISTANCE_F32>(), distances);
	EXPECT_EQ(pointCloud.getFieldValues<RETURN_TYPE_U8>(), returnType);
	EXPECT_EQ(pointCloud.getFieldValues<TIME_STAMP_F64>(), timeStamp);
}

TEST_P(PointCloudTest, setter_and_getter_with_paddings)
{
	int pointsCount = GetParam();

	PointCloud pointCloud = PointCloud(fieldsWithPaddings, pointsCount);
	EXPECT_EQ(pointCloud.getPointCount(), pointsCount);

	std::vector<Field<XYZ_F32>::type> pointCoord = generate(pointsCount, genCoord);
	std::vector<Field<IS_HIT_I32>::type> isHit = generate(pointsCount, genHalfHit);
	std::vector<Field<INTENSITY_F32>::type> intensity = generate(pointsCount, genIntensity);
	pointCloud.setFieldValues<XYZ_F32>(pointCoord);
	pointCloud.setFieldValues<IS_HIT_I32>(isHit);
	pointCloud.setFieldValues<INTENSITY_F32>(intensity);

	std::vector<Field<XYZ_F32>::type> outPointsCoord = pointCloud.getFieldValues<XYZ_F32>();
	for (int i = 0; i < pointsCount; i++) {
		EXPECT_EQ(outPointsCoord.at(i)[0], pointCoord.at(i)[0]);
		EXPECT_EQ(outPointsCoord.at(i)[1], pointCoord.at(i)[1]);
		EXPECT_EQ(outPointsCoord.at(i)[2], pointCoord.at(i)[2]);
	}
	EXPECT_EQ(pointCloud.getFieldValues<IS_HIT_I32>(), isHit);
	EXPECT_EQ(pointCloud.getFieldValues<INTENSITY_F32>(), intensity);
}

TEST_P(PointCloudTest, create_from_node)
{
	int pointsCount = GetParam();
	rgl_node_t usePointsNode = nullptr;

	std::vector<TestPointStruct> inPoints = generateTestPointsArray(pointsCount);
	ASSERT_RGL_SUCCESS(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), fields.data(), fields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

	PointCloud pointCloud = PointCloud::createFromNode(usePointsNode, fieldsWithPaddings);
	EXPECT_EQ(pointCloud.getPointCount(), pointsCount);
	std::vector<Field<XYZ_F32>::type> outPointsCoord = pointCloud.getFieldValues<XYZ_F32>();
	std::vector<Field<IS_HIT_I32>::type> outIsHit = pointCloud.getFieldValues<IS_HIT_I32>();
	std::vector<Field<INTENSITY_F32>::type> outIntensity = pointCloud.getFieldValues<INTENSITY_F32>();

	for (int i = 0; i < pointsCount; i++) {
		EXPECT_EQ(outPointsCoord.at(i)[0], inPoints.at(i).xyz[0]);
		EXPECT_EQ(outPointsCoord.at(i)[1], inPoints.at(i).xyz[1]);
		EXPECT_EQ(outPointsCoord.at(i)[2], inPoints.at(i).xyz[2]);
		EXPECT_EQ(outIsHit.at(i), inPoints.at(i).isHit);
		EXPECT_EQ(outIntensity.at(i), inPoints.at(i).intensity);
	}
}

TEST_P(PointCloudTest, equal_operator)
{
	int pointsCount = GetParam();
	rgl_node_t usePointsNode = nullptr;

	std::vector<TestPointStruct> inPoints = generateTestPointsArray(pointsCount);
	ASSERT_RGL_SUCCESS(
	    rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), fields.data(), fields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));
	PointCloud pointCloudFromNode = PointCloud::createFromNode(usePointsNode, fields);

	PointCloud pointCloud = PointCloud(fields, pointsCount);
	std::vector<Field<XYZ_F32>::type> pointCoord;
	std::vector<Field<IS_HIT_I32>::type> isHit;
	std::vector<Field<INTENSITY_F32>::type> intensity;
	pointCoord.reserve(pointsCount);
	isHit.reserve(pointsCount);
	intensity.reserve(pointsCount);
	for (int i = 0; i < pointsCount; i++) {
		pointCoord.push_back(inPoints.at(i).xyz);
		isHit.push_back(inPoints.at(i).isHit);
		intensity.push_back(inPoints.at(i).intensity);
	}
	pointCloud.setFieldValues<XYZ_F32>(pointCoord);
	pointCloud.setFieldValues<IS_HIT_I32>(isHit);
	pointCloud.setFieldValues<INTENSITY_F32>(intensity);

	EXPECT_TRUE(pointCloud == pointCloudFromNode);
}
