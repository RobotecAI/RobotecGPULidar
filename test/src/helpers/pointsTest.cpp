#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/mathHelpers.hpp>

class PointCloudTest : public ::testing::TestWithParam<int>
{
protected:
	std::vector<rgl_field_t> allNotDummyFields = {XYZ_VEC3_F32, IS_HIT_I32,  RAY_IDX_U32,  ENTITY_ID_I32,  INTENSITY_F32,
	                                              RING_ID_U16,  AZIMUTH_F32, DISTANCE_F32, RETURN_TYPE_U8, TIME_STAMP_F64};

	std::vector<rgl_field_t> fieldsWithPaddings = {PADDING_32,   XYZ_VEC3_F32,   PADDING_16,     IS_HIT_I32,  PADDING_8,
	                                               RAY_IDX_U32,  ENTITY_ID_I32,  INTENSITY_F32,  RING_ID_U16, AZIMUTH_F32,
	                                               DISTANCE_F32, RETURN_TYPE_U8, TIME_STAMP_F64, PADDING_16,  PADDING_32};

	std::vector<Field<XYZ_VEC3_F32>::type> pointCoord;
	std::vector<Field<IS_HIT_I32>::type> isHit;
	std::vector<Field<RAY_IDX_U32>::type> rayIdx;
	std::vector<Field<ENTITY_ID_I32>::type> entityId;
	std::vector<Field<INTENSITY_F32>::type> intensity;
	std::vector<Field<RING_ID_U16>::type> ringId;
	std::vector<Field<AZIMUTH_F32>::type> azimuth;
	std::vector<Field<DISTANCE_F32>::type> distance;
	std::vector<Field<RETURN_TYPE_U8>::type> returnType;
	std::vector<Field<TIME_STAMP_F64>::type> timeStamp;

#pragma pack(push, 1)
	struct TestPointStruct
	{
		Field<XYZ_VEC3_F32>::type xyz;
		Field<IS_HIT_I32>::type isHit;
		Field<RAY_IDX_U32>::type rayIdx;
		Field<ENTITY_ID_I32>::type entityId;
		Field<INTENSITY_F32>::type intensity;
		Field<RING_ID_U16>::type ringId;
		Field<AZIMUTH_F32>::type azimuth;
		Field<DISTANCE_F32>::type distance;
		Field<RETURN_TYPE_U8>::type returnType;
		Field<TIME_STAMP_F64>::type timeStamp;
	};
#pragma pack(pop)

	std::vector<TestPointStruct> generateTestPoints(int pointsCount)
	{
		std::vector<TestPointStruct> points;
		points.reserve(pointsCount);
		for (int i = 0; i < pointsCount; i++) {
			points.emplace_back(TestPointStruct{genCoord(i), genHalfHit(i), genRayIdx(i), genEntityId(i), genIntensity(i),
			                                    genRingId(i), genAzimuth(i), genDistance(i), genReturnType(i),
			                                    genTimeStamp(i)});
		}
		return points;
	}

	void generateFieldValues(int pointsCount)
	{
		pointCoord = generate(pointsCount, genCoord);
		isHit = generate(pointsCount, genHalfHit);
		rayIdx = generate(pointsCount, genRayIdx);
		entityId = generate(pointsCount, genEntityId);
		intensity = generate(pointsCount, genIntensity);
		ringId = generate(pointsCount, genRingId);
		azimuth = generate(pointsCount, genAzimuth);
		distance = generate(pointsCount, genDistance);
		returnType = generate(pointsCount, genReturnType);
		timeStamp = generate(pointsCount, genTimeStamp);
	}

	void setFieldValues(TestPointCloud& pointCloud)
	{
		pointCloud.setFieldValues<XYZ_VEC3_F32>(pointCoord);
		pointCloud.setFieldValues<IS_HIT_I32>(isHit);
		pointCloud.setFieldValues<RAY_IDX_U32>(rayIdx);
		pointCloud.setFieldValues<ENTITY_ID_I32>(entityId);
		pointCloud.setFieldValues<INTENSITY_F32>(intensity);
		pointCloud.setFieldValues<RING_ID_U16>(ringId);
		pointCloud.setFieldValues<AZIMUTH_F32>(azimuth);
		pointCloud.setFieldValues<DISTANCE_F32>(distance);
		pointCloud.setFieldValues<RETURN_TYPE_U8>(returnType);
		pointCloud.setFieldValues<TIME_STAMP_F64>(timeStamp);
	}

	void verifyResults(TestPointCloud& pointCloud, int pointsCount, Mat3x4f transform = Mat3x4f::identity())
	{
		std::vector<Field<XYZ_VEC3_F32>::type> outPointsCoord = pointCloud.getFieldValues<XYZ_VEC3_F32>();
		for (int i = 0; i < pointsCount; ++i) {
			pointCoord.at(i) = transform * pointCoord.at(i);
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
		EXPECT_EQ(pointCloud.getFieldValues<DISTANCE_F32>(), distance);
		EXPECT_EQ(pointCloud.getFieldValues<RETURN_TYPE_U8>(), returnType);
		EXPECT_EQ(pointCloud.getFieldValues<TIME_STAMP_F64>(), timeStamp);
	}
};

INSTANTIATE_TEST_SUITE_P(PointCloudTests, PointCloudTest, testing::Values(1, 100, maxGPUCoresTestCount),
                         [](const auto& info) { return "pointsCount_" + std::to_string(info.param); });

TEST_P(PointCloudTest, setter_and_getter_with_transform)
{
	int pointsCount = GetParam();
	Mat3x4f transform = Mat3x4f::fromRGL(complexTestTransform);

	TestPointCloud pointCloud = TestPointCloud(allNotDummyFields, pointsCount);
	EXPECT_EQ(pointCloud.getPointCount(), pointsCount);

	generateFieldValues(pointsCount);
	setFieldValues(pointCloud);
	pointCloud.transform(transform);

	verifyResults(pointCloud, pointsCount, transform);
}

TEST_P(PointCloudTest, setter_and_getter_with_paddings)
{
	int pointsCount = GetParam();

	TestPointCloud pointCloud = TestPointCloud(fieldsWithPaddings, pointsCount);
	EXPECT_EQ(pointCloud.getPointCount(), pointsCount);

	generateFieldValues(pointsCount);
	setFieldValues(pointCloud);

	verifyResults(pointCloud, pointsCount);
}

TEST_P(PointCloudTest, create_from_node)
{
	int pointsCount = GetParam();
	rgl_node_t usePointsNode = nullptr;

	std::vector<TestPointStruct> inPoints = generateTestPoints(pointsCount);
	generateFieldValues(pointsCount);

	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), allNotDummyFields.data(),
	                                              allNotDummyFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));
	TestPointCloud fromNodePointCloud = TestPointCloud::createFromNode(usePointsNode, allNotDummyFields);
	EXPECT_EQ(fromNodePointCloud.getPointCount(), pointsCount);

	verifyResults(fromNodePointCloud, pointsCount);
}

TEST_P(PointCloudTest, equal_operator)
{
	int pointsCount = GetParam();
	rgl_node_t usePointsNode = nullptr;

	std::vector<TestPointStruct> inPoints = generateTestPoints(pointsCount);
	generateFieldValues(pointsCount);

	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), allNotDummyFields.data(),
	                                              allNotDummyFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));
	TestPointCloud pointCloudFromNode = TestPointCloud::createFromNode(usePointsNode, allNotDummyFields);

	TestPointCloud expectedPointCloud = TestPointCloud(allNotDummyFields, pointsCount);
	setFieldValues(expectedPointCloud);

	EXPECT_TRUE(expectedPointCloud == pointCloudFromNode);
}
