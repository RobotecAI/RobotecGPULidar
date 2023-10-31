#include <helpers/commonHelpers.hpp>

class FormatPointsNodeTests : public RGLTest
{
protected:
	std::vector<rgl_field_t> fields;
	rgl_node_t formatNode;

	FormatPointsNodeTests()
	{
		fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
		formatNode = nullptr;
	}
};

// TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields.
TEST_F(FormatPointsNodeTests, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");
}

TEST_F(FormatPointsNodeTests, invalid_argument_fields)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, nullptr, 0), "fields != nullptr");
}

TEST_F(FormatPointsNodeTests, invalid_argument_field_count)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, fields.data(), 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTests, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
	ASSERT_THAT(formatNode, testing::NotNull());

	// If (*formatNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
}

//TEST_F(FormatPointsNodeTest, error_when_invalid_field_request)
//{
//	exampleFields.push_back(RGL_FIELD_DYNAMIC_FORMAT);
//	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNodes.at(0), exampleFields.data(), exampleFields.size()),
//	                            "cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");
//}
//
//TEST_F(FormatPointsNodeTest, format_when_empty_point_cloud_input)
//{
//	rgl_node_t emptyPointCloudOutputNode = simulateEmptyPointCloudOutputNode(exampleFields);
//
//	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), exampleFields.data(), exampleFields.size()));
//	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(emptyPointCloudOutputNode, formatNodes.at(0)));
//	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(0)));
//
//	int32_t outCount, outSizeOf;
//	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
//	EXPECT_EQ(outCount, 0);
//
//	std::size_t sizeOfExampleFields = 0;
//	for (auto field : exampleFields) {
//		sizeOfExampleFields += getFieldSize(field);
//	}
//	EXPECT_EQ(outSizeOf, sizeOfExampleFields);
//
//	// Check if the contents of outData have changed (they should not have)
//	std::vector<::Field<XYZ_VEC3_F32>::type> outData{
//	    { 2.0f,  3.0f,  5.0f},
//        { 7.0f, 11.0f, 13.0f},
//        {17.0f, 19.0f, 23.0f}
//    };
//	auto outDataCopy = outData;
//
//	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), RGL_FIELD_XYZ_VEC3_F32, outData.data()));
//	ASSERT_EQ(outData.size(), outDataCopy.size());
//	for (int i = 0; i < outData.size(); ++i) {
//		EXPECT_EQ(outData.at(i)[0], outDataCopy.at(i)[0]);
//		EXPECT_EQ(outData.at(i)[1], outDataCopy.at(i)[1]);
//		EXPECT_EQ(outData.at(i)[2], outDataCopy.at(i)[2]);
//	}
//}

// TEST_P(FormatPointsNodeTest, should_properly_align_output_size)
// {
// 	int pointsCount = GetParam();
// 	std::vector<PointWithoutPadding> points = generatePointsWithoutPaddings(pointsCount);
// 	auto fields = PointWithoutPadding::getPointFields();
// 	auto fieldsToFormat = PointQuadrPadding::getPointFields();

// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), fields.data(), fields.size()));
// 	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fieldsToFormat.data(), fieldsToFormat.size()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(0)));
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(0)));

// 	int32_t outCount, outSizeOf;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	EXPECT_EQ(outCount, pointsCount);
// 	EXPECT_EQ(outSizeOf, getPointSize(fieldsToFormat));
// }

// TEST_P(FormatPointsNodeTest, format_node_results_data_should_be_the_same_as_from_yield_node)
// {
// 	int pointsCount = GetParam();
// 	auto points = generatePointsWithoutPaddings(pointsCount);
// 	auto fields = PointWithoutPadding::getPointFields();
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), fields.data(), fields.size()));

// 	// Using yield points node
// 	ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldPointsNode, fields.data(), fields.size()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, yieldPointsNode));
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(yieldPointsNode));

// 	int32_t outPointsCount, outPointsSize, outIsHitsCount, outIsHitsSize, outDistancesCount, outDistancesSize;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, XYZ_F32, &outPointsCount, &outPointsSize));
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, IS_HIT_I32, &outIsHitsCount, &outIsHitsSize));
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, DISTANCE_F32, &outDistancesCount, &outDistancesSize));
// 	ASSERT_EQ(outPointsCount, pointsCount);
// 	ASSERT_EQ(outIsHitsCount, pointsCount);
// 	ASSERT_EQ(outDistancesCount, pointsCount);

// 	std::vector<::Field<XYZ_F32>::type> outPoints(outPointsCount);
// 	std::vector<::Field<IS_HIT_I32>::type> outIsHits(outIsHitsCount);
// 	std::vector<::Field<DISTANCE_F32>::type> outDistances(outDistancesCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, XYZ_F32, outPoints.data()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, IS_HIT_I32, outIsHits.data()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, DISTANCE_F32, outDistances.data()));
// 	ASSERT_EQ(outPoints.size(), pointsCount);
// 	ASSERT_EQ(outIsHits.size(), pointsCount);
// 	ASSERT_EQ(outDistances.size(), pointsCount);

// 	// Using format points node
// 	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fields.data(), fields.size()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(usePointsNode, yieldPointsNode));
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(0)));
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(0)));

// 	int32_t outCount, outSizeOf;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fields));

// 	std::vector<PointWithoutPadding> outFormatPoints(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));
// 	ASSERT_EQ(outFormatPoints.size(), pointsCount);

// 	// Compare yield points output and format points output
// 	for (int i = 0; i < outCount; ++i) {
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[0], points.at(i).xyz[0]);
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[1], points.at(i).xyz[1]);
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[2], points.at(i).xyz[2]);
// 		ASSERT_EQ(outFormatPoints.at(i).isHit, points.at(i).isHit);
// 		ASSERT_EQ(outFormatPoints.at(i).distance, points.at(i).distance);

// 		EXPECT_EQ(outPoints.at(i)[0], outFormatPoints.at(i).xyz[0]);
// 		EXPECT_EQ(outPoints.at(i)[1], outFormatPoints.at(i).xyz[1]);
// 		EXPECT_EQ(outPoints.at(i)[2], outFormatPoints.at(i).xyz[2]);
// 		EXPECT_EQ(outIsHits.at(i), outFormatPoints.at(i).isHit);
// 		EXPECT_EQ(outDistances.at(i), outFormatPoints.at(i).distance);
// 	}

// 	// TODO Consider calculating the numerical loss when converting AoS (format) to SoA (yield) as part of testing
// }

// TEST_P(FormatPointsNodeTest, multiple_format_should_not_change_data)
// {
// 	int pointsCount = GetParam();
// 	auto points = generatePointsWithoutPaddings(pointsCount);
// 	auto fields = PointWithoutPadding::getPointFields();
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), fields.data(), fields.size()));

// 	for (auto& formatNode : formatNodes) {
// 		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
// 	}
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(0)));
// 	for (int i = 1; i < formatNodes.size(); ++i) {
// 		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatNodes.at(i - 1), formatNodes.at(i)));
// 	}
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

// 	// Compare results from first and last format node
// 	int32_t outCount, outSizeOf;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fields));

// 	std::vector<PointWithoutPadding> outFormatPoints(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));
// 	ASSERT_EQ(outFormatPoints.size(), pointsCount);

// 	ASSERT_RGL_SUCCESS(
// 	    rgl_graph_get_result_size(formatNodes.at(formatNodes.size() - 1), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fields));

// 	std::vector<PointWithoutPadding> outLastFormatPoints(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(formatNodes.size() - 1), RGL_FIELD_DYNAMIC_FORMAT,
// 	                                             outLastFormatPoints.data()));
// 	ASSERT_EQ(outLastFormatPoints.size(), pointsCount);

// 	ASSERT_EQ(outFormatPoints.size(), outLastFormatPoints.size());
// 	for (int i = 0; i < outFormatPoints.size(); ++i) {
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[0], points.at(i).xyz[0]);
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[1], points.at(i).xyz[1]);
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[2], points.at(i).xyz[2]);
// 		ASSERT_EQ(outFormatPoints.at(i).isHit, points.at(i).isHit);
// 		ASSERT_EQ(outFormatPoints.at(i).distance, points.at(i).distance);

// 		EXPECT_EQ(outFormatPoints.at(i).xyz[0], outLastFormatPoints.at(i).xyz[0]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[1], outLastFormatPoints.at(i).xyz[1]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[2], outLastFormatPoints.at(i).xyz[2]);
// 		EXPECT_EQ(outFormatPoints.at(i).isHit, outLastFormatPoints.at(i).isHit);
// 		EXPECT_EQ(outFormatPoints.at(i).distance, outLastFormatPoints.at(i).distance);
// 	}
// }

// TEST_P(FormatPointsNodeTest, multiple_changing_format_should_not_change_data)
// {
// 	int pointsCount = GetParam();
// 	auto points = generatePointsWithoutPaddings(pointsCount);
// 	auto fields = PointWithoutPadding::getPointFields();
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), fields.data(), fields.size()));

// 	std::vector<std::vector<rgl_field_t>> fieldsToFormat;
// 	fieldsToFormat.push_back(PointWithoutPadding::getPointFields());
// 	fieldsToFormat.push_back(PointWithPadding::getPointFields());
// 	fieldsToFormat.push_back(PointDoublePadding::getPointFields());
// 	fieldsToFormat.push_back(PointTriplePadding::getPointFields());
// 	fieldsToFormat.push_back(PointQuadrPadding::getPointFields());

// 	for (int i = 0; i < fieldsToFormat.size(); ++i) {
// 		ASSERT_RGL_SUCCESS(
// 		    rgl_node_points_format(&formatNodes.at(i), fieldsToFormat.at(i).data(), fieldsToFormat.at(i).size()));
// 	}

// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(0)));
// 	for (int i = 1; i < fieldsToFormat.size(); ++i) {
// 		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatNodes.at(i - 1), formatNodes.at(i)));
// 	}
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

// 	int32_t outCount, outSizeOf;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fieldsToFormat.at(0)));

// 	std::vector<PointWithoutPadding> outFormatPoints(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));
// 	ASSERT_EQ(outFormatPoints.size(), pointsCount);

// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(1), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fieldsToFormat.at(1)));

// 	std::vector<PointWithPadding> outFormatPoints1(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(1), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints1.data()));
// 	ASSERT_EQ(outFormatPoints1.size(), pointsCount);

// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(2), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fieldsToFormat.at(2)));

// 	std::vector<PointDoublePadding> outFormatPoints2(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(2), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints2.data()));
// 	ASSERT_EQ(outFormatPoints2.size(), pointsCount);

// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(3), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fieldsToFormat.at(3)));

// 	std::vector<PointTriplePadding> outFormatPoints3(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(3), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints3.data()));
// 	ASSERT_EQ(outFormatPoints3.size(), pointsCount);

// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(4), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fieldsToFormat.at(4)));

// 	std::vector<PointQuadrPadding> outFormatPoints4(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(4), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints4.data()));
// 	ASSERT_EQ(outFormatPoints4.size(), pointsCount);

// 	for (int i = 0; i < outFormatPoints.size(); ++i) {
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[0], points.at(i).xyz[0]);
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[1], points.at(i).xyz[1]);
// 		ASSERT_EQ(outFormatPoints.at(i).xyz[2], points.at(i).xyz[2]);
// 		ASSERT_EQ(outFormatPoints.at(i).isHit, points.at(i).isHit);
// 		ASSERT_EQ(outFormatPoints.at(i).distance, points.at(i).distance);

// 		EXPECT_EQ(outFormatPoints.at(i).xyz[0], outFormatPoints1.at(i).xyz[0]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[1], outFormatPoints1.at(i).xyz[1]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[2], outFormatPoints1.at(i).xyz[2]);
// 		EXPECT_EQ(outFormatPoints.at(i).isHit, outFormatPoints1.at(i).isHit);
// 		EXPECT_EQ(outFormatPoints.at(i).distance, outFormatPoints1.at(i).distance);

// 		EXPECT_EQ(outFormatPoints.at(i).xyz[0], outFormatPoints2.at(i).xyz[0]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[1], outFormatPoints2.at(i).xyz[1]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[2], outFormatPoints2.at(i).xyz[2]);
// 		EXPECT_EQ(outFormatPoints.at(i).isHit, outFormatPoints2.at(i).isHit);
// 		EXPECT_EQ(outFormatPoints.at(i).distance, outFormatPoints2.at(i).distance);

// 		EXPECT_EQ(outFormatPoints.at(i).xyz[0], outFormatPoints3.at(i).xyz[0]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[1], outFormatPoints3.at(i).xyz[1]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[2], outFormatPoints3.at(i).xyz[2]);
// 		EXPECT_EQ(outFormatPoints.at(i).isHit, outFormatPoints3.at(i).isHit);
// 		EXPECT_EQ(outFormatPoints.at(i).distance, outFormatPoints3.at(i).distance);

// 		EXPECT_EQ(outFormatPoints.at(i).xyz[0], outFormatPoints4.at(i).xyz[0]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[1], outFormatPoints4.at(i).xyz[1]);
// 		EXPECT_EQ(outFormatPoints.at(i).xyz[2], outFormatPoints4.at(i).xyz[2]);
// 		EXPECT_EQ(outFormatPoints.at(i).isHit, outFormatPoints4.at(i).isHit);
// 		EXPECT_EQ(outFormatPoints.at(i).distance, outFormatPoints4.at(i).distance);
// 	}
// }

// TEST_P(FormatPointsNodeTest, paddings_alone)
// {
// 	int pointsCount = GetParam();
// 	auto points = generatePointsWithoutPaddings(pointsCount);
// 	auto fields = PointWithoutPadding::getPointFields();
// 	std::vector<rgl_field_t> fieldsToFormat = {PADDING_8, PADDING_16, PADDING_32, PADDING_16, PADDING_8};
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), fields.data(), fields.size()));

// 	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fieldsToFormat.data(), fieldsToFormat.size()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(0)));
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(0)));

// 	int32_t outCount, outSizeOf;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	EXPECT_EQ(outCount, pointsCount);
// 	EXPECT_EQ(outSizeOf, getPointSize(fieldsToFormat));
// }

// TEST_P(FormatPointsNodeTest, duplicated_fields)
// {
// 	int pointsCount = GetParam();
// 	auto points = generatePointsWithoutPaddings(pointsCount);
// 	auto fields = PointWithoutPadding::getPointFields();
// 	std::vector<rgl_field_t> fieldsToFormat = {XYZ_F32, XYZ_F32, XYZ_F32, XYZ_F32, XYZ_F32};
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, points.data(), points.size(), fields.data(), fields.size()));

// 	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fieldsToFormat.data(), fieldsToFormat.size()));
// 	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(0)));
// 	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(0)));

// 	int32_t outCount, outSizeOf;
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
// 	ASSERT_EQ(outCount, pointsCount);
// 	ASSERT_EQ(outSizeOf, getPointSize(fieldsToFormat));

// 	std::vector<Field<XYZ_F32>::type> outData(outCount);
// 	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), XYZ_F32, outData.data()));
// 	ASSERT_EQ(outData.size(), pointsCount);
// 	for (int i = 0; i < outCount; ++i) {
// 		EXPECT_EQ(outData.at(i)[0], points.at(i).xyz[0]);
// 		EXPECT_EQ(outData.at(i)[1], points.at(i).xyz[1]);
// 		EXPECT_EQ(outData.at(i)[2], points.at(i).xyz[2]);
// 	}
// }

// TEST_F(FormatPointsNodeTest, request_for_field_that_points_do_not_have)
// {
// 	int pointsCount = 1;
// 	auto isHitPoint = isHitAlonePoint{1};
// 	auto fields = isHitAlonePoint::getPointFields();
// 	std::vector<std::vector<rgl_field_t>> fieldsToFormat = {{XYZ_F32},       {RAY_IDX_U32},    {ENTITY_ID_I32},
// 	                                                        {INTENSITY_F32}, {RING_ID_U16},    {AZIMUTH_F32},
// 	                                                        {DISTANCE_F32},  {RETURN_TYPE_U8}, {TIME_STAMP_F64}};
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, &isHitPoint, pointsCount, fields.data(), fields.size()));

// 	for (int i = 0; i < fieldsToFormat.size(); ++i) {
// 		ASSERT_RGL_SUCCESS(
// 		    rgl_node_points_format(&formatNodes.at(i), fieldsToFormat.at(i).data(), fieldsToFormat.at(i).size()));
// 	}

// 	for (int i = 0; i < fieldsToFormat.size(); ++i) {
// 		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(i)));
// 		EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(formatNodes.at(i)), toString(fieldsToFormat.at(i).at(0)));
// 		ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(usePointsNode, formatNodes.at(i)));
// 	}

// 	auto xyzPoint = xyzAlonePoint{
// 	    {1.0f, 2.0f, 3.0f}
//     };
// 	fields = xyzAlonePoint::getPointFields();
// 	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, &xyzPoint, pointsCount, fields.data(), fields.size()));

// 	for (int i = 1; i < fieldsToFormat.size(); ++i) {
// 		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNodes.at(i)));
// 		EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(formatNodes.at(i)), toString(fieldsToFormat.at(i).at(0)));
// 		ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(usePointsNode, formatNodes.at(i)));
// 	}
// }
