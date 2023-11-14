#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <api/apiCommon.hpp>
#include <RGLFields.hpp>

#include <numeric>
#include <random>

static constexpr int MULTIPLE_FORMATS_COUNT = 15;

class FormatPointsNodeTest : public RGLTestWithParam<std::tuple<int, int>>
{
public:
	static void SetUpTestCase()
	{
		static std::once_flag flag;
		std::call_once(flag, []() { fmt::print(stderr, "Format Points Node Test random seed: {}\n", randomSeed); });
	}

protected:
	rgl_node_t formatNode = nullptr;

	void runGraphWithAssertions(rgl_node_t& inNode, const std::vector<rgl_field_t>& fieldsToFormat)
	{
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fieldsToFormat.data(), fieldsToFormat.size()));
		ASSERT_THAT(formatNode, testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, formatNode));
		ASSERT_RGL_SUCCESS(rgl_graph_run(inNode));
	}
};

INSTANTIATE_TEST_SUITE_P(FormatPointsNodeTests, FormatPointsNodeTest,
                         testing::Combine(testing::Values(1, 100, maxGPUCoresTestCount), testing::Values(0, 1, 5, 10)),
                         [](const auto& info) {
	                         return "pointsCount_" + std::to_string(std::get<0>(info.param)) + "paddingCount_" +
	                                std::to_string(std::get<1>(info.param));
                         });

TEST_F(FormatPointsNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");
}

TEST_F(FormatPointsNodeTest, invalid_argument_fields)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, nullptr, 0), "fields != nullptr");
}

TEST_F(FormatPointsNodeTest, invalid_argument_field_count)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, allNotDummyFields.data(), 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allNotDummyFields.data(), allNotDummyFields.size()));
	ASSERT_THAT(formatNode, testing::NotNull());

	// If (*formatNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allNotDummyFields.data(), allNotDummyFields.size()));
}

TEST_F(FormatPointsNodeTest, error_when_invalid_field_request)
{
	rgl_field_t invalidFieldToFormat[1] = {RGL_FIELD_DYNAMIC_FORMAT};
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, invalidFieldToFormat, sizeof(invalidFieldToFormat)),
	                            "cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");
}

TEST_F(FormatPointsNodeTest, format_when_empty_point_cloud_input)
{
	rgl_node_t emptyPointCloudOutputNode = nullptr;
	createOrUpdateNode<EmptyNode>(&emptyPointCloudOutputNode);

	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allNotDummyFields.data(), allNotDummyFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(emptyPointCloudOutputNode, formatNode));
	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

	// When creating outPointCloud, the createFromFormatNode method checks that the outSize matches the field size
	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, allNotDummyFields);
	EXPECT_EQ(outPointCloud.getPointCount(), 0);

	// Check if the contents of outData have changed (they should not have)
	std::vector<::Field<XYZ_VEC3_F32>::type> outData{
	    { 2.0f,  3.0f,  5.0f},
        { 7.0f, 11.0f, 13.0f},
        {17.0f, 19.0f, 23.0f}
    };
	auto outDataCopy = outData;

	// Verify that the data is unchanged; this is a sanity check to make sure that the data is not overwritten
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_XYZ_VEC3_F32, outData.data()));
	ASSERT_EQ(outData.size(), outDataCopy.size());
	for (int i = 0; i < outData.size(); ++i) {
		EXPECT_EQ(outData.at(i)[0], outDataCopy.at(i)[0]);
		EXPECT_EQ(outData.at(i)[1], outDataCopy.at(i)[1]);
		EXPECT_EQ(outData.at(i)[2], outDataCopy.at(i)[2]);
	}
}

TEST_P(FormatPointsNodeTest, should_properly_align_output_size_and_not_change_data)
{
	int pointsCount = std::get<0>(GetParam());
	int numberOfPaddings = std::get<1>(GetParam());

	auto fieldsToFormat = generateRandomStaticFieldsVector(numberOfPaddings);
	TestPointCloud inPointCloud = TestPointCloud(fieldsToFormat, pointsCount);
	rgl_node_t inNode = inPointCloud.createUsePointsNode();

	runGraphWithAssertions(inNode, fieldsToFormat);

	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, fieldsToFormat);

	EXPECT_EQ(outPointCloud, inPointCloud);
}

TEST_P(FormatPointsNodeTest, multiple_format_should_not_change_data)
{
	int pointsCount = std::get<0>(GetParam());
	int numberOfPaddings = std::get<1>(GetParam());

	auto fieldsToFormat = generateRandomStaticFieldsVector(numberOfPaddings);
	TestPointCloud pointCloud = TestPointCloud(fieldsToFormat, pointsCount);
	rgl_node_t inNode = pointCloud.createUsePointsNode();

	std::vector<rgl_node_t> formatNodes(MULTIPLE_FORMATS_COUNT);
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fieldsToFormat.data(), fieldsToFormat.size()));
	ASSERT_THAT(formatNodes.at(0), testing::NotNull());

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, formatNodes.at(0)));
	for (int i = 1; i < MULTIPLE_FORMATS_COUNT; ++i) {
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(i), fieldsToFormat.data(), fieldsToFormat.size()));
		ASSERT_THAT(formatNodes.at(i), testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatNodes.at(i - 1), formatNodes.at(i)));
	}
	ASSERT_RGL_SUCCESS(rgl_graph_run(inNode));

	auto outPointCloud = TestPointCloud::createFromFormatNode(formatNodes.at(MULTIPLE_FORMATS_COUNT - 1), fieldsToFormat);
	EXPECT_EQ(pointCloud, outPointCloud);
}

TEST_P(FormatPointsNodeTest, multiple_changing_format_should_not_change_data)
{
	int pointsCount = std::get<0>(GetParam());
	int numberOfPaddings = std::get<1>(GetParam());

	std::vector<std::vector<rgl_field_t>> fieldsToFormat(MULTIPLE_FORMATS_COUNT);
	for (int i = 0; i < MULTIPLE_FORMATS_COUNT; ++i) {
		fieldsToFormat.at(i) = generateRandomStaticFieldsVector(numberOfPaddings);
	}

	TestPointCloud pointCloud = TestPointCloud(fieldsToFormat.at(MULTIPLE_FORMATS_COUNT - 1), pointsCount);
	rgl_node_t inNode = pointCloud.createUsePointsNode();

	std::vector<rgl_node_t> formatNodes(MULTIPLE_FORMATS_COUNT);
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fieldsToFormat.at(0).data(), fieldsToFormat.at(0).size()));
	ASSERT_THAT(formatNodes.at(0), testing::NotNull());

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, formatNodes.at(0)));
	for (int i = 1; i < MULTIPLE_FORMATS_COUNT; ++i) {
		ASSERT_RGL_SUCCESS(
		    rgl_node_points_format(&formatNodes.at(i), fieldsToFormat.at(i).data(), fieldsToFormat.at(i).size()));
		ASSERT_THAT(formatNodes.at(i), testing::NotNull());
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatNodes.at(i - 1), formatNodes.at(i)));
	}
	ASSERT_RGL_SUCCESS(rgl_graph_run(inNode));

	auto outPointCloud = TestPointCloud::createFromFormatNode(formatNodes.at(MULTIPLE_FORMATS_COUNT - 1),
	                                                          fieldsToFormat.at(MULTIPLE_FORMATS_COUNT - 1));

	EXPECT_EQ(pointCloud, outPointCloud);
}

TEST_P(FormatPointsNodeTest, paddings_alone)
{
	int pointsCount = std::get<0>(GetParam());

	TestPointCloud pointCloud = TestPointCloud(availablePaddings, pointsCount);
	rgl_node_t inNode = pointCloud.createUsePointsNode();

	runGraphWithAssertions(inNode, availablePaddings);

	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, availablePaddings);
	EXPECT_EQ(outPointCloud.getPointCount(), pointsCount);
}

TEST_P(FormatPointsNodeTest, duplicated_fields)
{
	int pointsCount = std::get<0>(GetParam());
	std::vector<rgl_field_t> fieldsToFormat = {XYZ_VEC3_F32, XYZ_VEC3_F32, XYZ_VEC3_F32, DISTANCE_F32, DISTANCE_F32};

	TestPointCloud inPointCloud = TestPointCloud(fieldsToFormat, pointsCount);
	rgl_node_t inNode = inPointCloud.createUsePointsNode();

	runGraphWithAssertions(inNode, fieldsToFormat);

	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, fieldsToFormat);

	EXPECT_EQ(outPointCloud, inPointCloud);
}

TEST_P(FormatPointsNodeTest, request_for_field_that_points_do_not_have)
{
	int pointsCount = std::get<0>(GetParam());

	// Removing first field from allNotDummyFields
	std::vector<rgl_field_t> pointFields = allNotDummyFields;
	rgl_node_t inNode = nullptr;

	for (int i = 0; i < allNotDummyFields.size(); ++i) {
		rgl_field_t fieldToRemove = allNotDummyFields[i];
		pointFields.erase(std::find(pointFields.begin(), pointFields.end(), fieldToRemove));

		TestPointCloud pointCloud = TestPointCloud(pointFields, pointsCount);
		inNode = pointCloud.createUsePointsNode();

		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allNotDummyFields.data(), allNotDummyFields.size()));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, formatNode));
		EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(inNode), fmt::format("requires {} to be present", toString(fieldToRemove)));

		pointFields.emplace_back(fieldToRemove);
		rgl_graph_node_remove_child(inNode, formatNode);
	}
}
