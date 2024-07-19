#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>
#include <helpers/testPointCloud.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/lidarHelpers.hpp>

#include <Time.hpp>
#include <math/Mat3x4f.hpp>
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

	const std::vector<rgl_field_t> allRealFields = getAllRealFieldsVector();
	const std::vector<rgl_field_t> allPaddings = getAllPaddingsVector();

	void runGraphWithAssertions(rgl_node_t inNode, const std::vector<rgl_field_t>& fieldsToFormat)
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
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, allRealFields.data(), 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allRealFields.data(), allRealFields.size()));
	ASSERT_THAT(formatNode, testing::NotNull());

	// If (*formatNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allRealFields.data(), allRealFields.size()));
}

TEST_F(FormatPointsNodeTest, error_when_invalid_field_request)
{
	rgl_field_t invalidFieldToFormat[1] = {RGL_FIELD_DYNAMIC_FORMAT};
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, invalidFieldToFormat, sizeof(invalidFieldToFormat)),
	                            "cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");
}

TEST_F(FormatPointsNodeTest, should_format_empty_input)
{
	rgl_node_t emptyPointCloudOutputNode = nullptr;
	createOrUpdateNode<EmptyNode>(&emptyPointCloudOutputNode);

	ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allRealFields.data(), allRealFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(emptyPointCloudOutputNode, formatNode));
	ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

	// When creating outPointCloud, the createFromFormatNode method checks that the outSize matches the field size
	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, allRealFields);
	EXPECT_EQ(outPointCloud.getPointCount(), 0);

	// Check if the contents of outData have changed (they should not have)
	std::vector<::Field<XYZ_VEC3_F32>::type> outData{
	    { 2.0f,  3.0f,  5.0f},
	    { 7.0f, 11.0f, 13.0f},
	    {17.0f, 19.0f, 23.0f},
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

TEST_P(FormatPointsNodeTest, should_format_and_not_change_data)
{
	int pointsCount = std::get<0>(GetParam());
	int numberOfPaddings = std::get<1>(GetParam());

	auto fieldsToFormat = generateRandomFieldsVector(numberOfPaddings);
	TestPointCloud inPointCloud = TestPointCloud(fieldsToFormat, pointsCount);
	rgl_node_t inNode = inPointCloud.createUsePointsNode();

	runGraphWithAssertions(inNode, fieldsToFormat);

	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, fieldsToFormat);

	EXPECT_EQ(outPointCloud, inPointCloud);
}

TEST_P(FormatPointsNodeTest, multiple_changing_format_should_not_change_data)
{
	int pointsCount = std::get<0>(GetParam());
	int numberOfPaddings = std::get<1>(GetParam());

	std::vector<std::vector<rgl_field_t>> fieldsToFormat(MULTIPLE_FORMATS_COUNT);
	for (int i = 0; i < MULTIPLE_FORMATS_COUNT; ++i) {
		fieldsToFormat.at(i) = generateRandomFieldsVector(numberOfPaddings);
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

TEST_F(FormatPointsNodeTest, paddings_alone)
{
	int pointsCount = maxGPUCoresTestCount;

	TestPointCloud pointCloud = TestPointCloud(allPaddings, pointsCount);
	rgl_node_t inNode = pointCloud.createUsePointsNode();

	runGraphWithAssertions(inNode, allPaddings);

	// When creating outPointCloud, the createFromFormatNode method checks that the outSize matches the field size
	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, allPaddings);
	EXPECT_EQ(outPointCloud.getPointCount(), pointsCount);
}

TEST_F(FormatPointsNodeTest, duplicated_fields)
{
	int pointsCount = maxGPUCoresTestCount;

	// Duplicate every real field
	std::vector<rgl_field_t> fieldsToFormat;
	for (auto field : allRealFields) {
		fieldsToFormat.emplace_back(field);
		fieldsToFormat.emplace_back(field);
	}

	TestPointCloud inPointCloud = TestPointCloud(fieldsToFormat, pointsCount);
	rgl_node_t inNode = inPointCloud.createUsePointsNode();

	runGraphWithAssertions(inNode, fieldsToFormat);

	TestPointCloud outPointCloud = TestPointCloud::createFromFormatNode(formatNode, fieldsToFormat);

	EXPECT_EQ(outPointCloud, inPointCloud);
}

TEST_F(FormatPointsNodeTest, request_for_field_that_points_do_not_have)
{
	int pointsCount = maxGPUCoresTestCount;

	std::vector<rgl_field_t> pointFields = allRealFields;
	rgl_node_t inNode = nullptr;

	for (int i = 0; i < allRealFields.size(); ++i) {
		rgl_field_t fieldToRemove = allRealFields[i];
		pointFields.erase(std::find(pointFields.begin(), pointFields.end(), fieldToRemove));

		TestPointCloud pointCloud = TestPointCloud(pointFields, pointsCount);
		inNode = pointCloud.createUsePointsNode();

		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, allRealFields.data(), allRealFields.size()));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(inNode, formatNode));
		EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(inNode), fmt::format("requires {} to be present", toString(fieldToRemove)));

		pointFields.emplace_back(fieldToRemove);
		rgl_graph_node_remove_child(inNode, formatNode);
	}
}

TEST_F(FormatPointsNodeTest, validate_results_after_changing_format)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, format = nullptr;

	// The cube located in 0,0,0 with width equals 1, rays shoot in perpendicular direction
	constexpr float EXPECTED_HITPOINT_Z = 1.0f;
	constexpr float EXPECTED_RAY_DISTANCE = 1.0f;
	std::vector<rgl_mat3x4f> rays = {Mat3x4f::TRS({0, 0, 0}).toRGL(), Mat3x4f::TRS({0.1, 0, 0}).toRGL(),
	                                 Mat3x4f::TRS({0.2, 0, 0}).toRGL(), Mat3x4f::TRS({0.3, 0, 0}).toRGL(),
	                                 Mat3x4f::TRS({0.4, 0, 0}).toRGL()};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::identity().toRGL();
	std::vector<rgl_field_t> formatFields = {XYZ_VEC3_F32, PADDING_32};
	struct FormatStruct
	{
		Field<XYZ_VEC3_F32>::type xyz;
		Field<PADDING_32>::type padding;
	} formatStruct;

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData(outCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], rays[i].value[0][3], EPSILON_F);
		EXPECT_NEAR(formatData[i].xyz[1], rays[i].value[1][3], EPSILON_F);
		EXPECT_NEAR(formatData[i].xyz[2], EXPECTED_HITPOINT_Z, EPSILON_F);
	}

	// Test if fields update is propagated over graph properly
	formatFields.emplace_back(DISTANCE_F32); // Add distance field
	formatFields.emplace_back(PADDING_32);   // Align to 8 bytes
	struct FormatStructExtended : public FormatStruct
	{
		Field<DISTANCE_F32>::type distance;
		Field<PADDING_32>::type padding2; // Align to 8 bytes
	} formatStructEx;

	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	outCount = -1; // reset variables
	outSizeOf = -1;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStructEx));

	std::vector<FormatStructExtended> formatDataEx{(size_t) outCount};
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatDataEx.data()));

	for (int i = 0; i < formatDataEx.size(); ++i) {
		EXPECT_NEAR(formatDataEx[i].xyz[0], rays[i].value[0][3], EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].xyz[1], rays[i].value[1][3], EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].xyz[2], EXPECTED_HITPOINT_Z, EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].distance, EXPECTED_RAY_DISTANCE, EPSILON_F);
	}
}

TEST_F(FormatPointsNodeTest, changing_required_fields_between_runs)
{
	spawnCubeOnScene(Mat3x4f::identity());

	rgl_node_t useRays = nullptr, raytrace = nullptr, compact = nullptr, format = nullptr;

	std::vector<std::vector<rgl_field_t>> subsequentFormatFields = {
	    {RGL_FIELD_XYZ_VEC3_F32 },
	    { RGL_FIELD_XYZ_VEC3_F32,RGL_FIELD_INTENSITY_F32, RGL_FIELD_ENTITY_ID_I32},
	    { RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_INTENSITY_F32},
	    { RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_INTENSITY_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_IS_HIT_I32},
	    { RGL_FIELD_XYZ_VEC3_F32,  RGL_FIELD_DISTANCE_F32},
	};

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&format, subsequentFormatFields[0].data(), subsequentFormatFields[0].size()));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format));

	// 1st method of changing required fields: update node with fields as a parameter
	for (auto&& formatFields : subsequentFormatFields) {
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace)); // Run with dirty nodes
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace)); // Run with valid nodes
	}

	// 2nd method of changing required fields: remove child node with user-defined fields requirements
	for (auto&& formatFields : subsequentFormatFields) {
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));
		ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(compact, format)); // Remove node with extra fields in the pipeline
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format)); // Restore connection for the next loop iteration
	}
}