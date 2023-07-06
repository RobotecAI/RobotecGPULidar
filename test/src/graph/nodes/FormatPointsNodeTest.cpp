#include <utils.hpp>
#include <DistanceFieldTestHelper.hpp>
#include <UsePointsNodeHelper.hpp>

class FormatPointsNodeTests : public RGLTest, public DistanceFieldTestHelper{
protected:
    std::vector<rgl_field_t> fields;
    rgl_node_t formatNode;

    FormatPointsNodeTests() {
        fields = {XYZ_F32, PADDING_32};
        formatNode = nullptr;
    }

    struct FormatStruct
    {
        ::Field<XYZ_F32>::type xyz;
        ::Field<PADDING_32>::type padding;
    } formatStruct;
};

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

TEST_F(FormatPointsNodeTests, invalid_field_request) // done
{
    fields.push_back(TIME_STAMP_F64);

    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, 100).toRGL();
    prepareSceneWithCube(cubePoseTf);
    rayTf.push_back(Mat3x4f::identity().toRGL());

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNode));

    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(useRaysNode), "cannot get time from scene");

    fields.pop_back();
    fields.push_back(RING_ID_U16);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));

    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(useRaysNode), "cannot get ring ids");

    fields.pop_back();
    fields.push_back(RGL_FIELD_DYNAMIC_FORMAT);
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, fields.data(), fields.size()), "cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");

    fields.pop_back();
    fields.push_back((rgl_field_t)12);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_run(formatNode), "unknown RGL field");
}

TEST_F(FormatPointsNodeTests, empty_point_cloud_input)
{
    RGLTestUsePointsNodeHelper<TestPointIsHit> helper;
    rgl_node_t compactNode = helper.simulateEmptyPointCloudOutputNode();

    fields.push_back(IS_HIT_I32);
	fields.push_back(PADDING_32);

	struct FormatStructExtended : public  FormatStruct
	{
		Field<IS_HIT_I32>::type isHit;
		Field<PADDING_32>::type padding2;
	} formatStructEx;

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));

    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));

    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

    int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, 0);
	EXPECT_EQ(outSizeOf, sizeof(formatStructEx));
}

TEST_F(FormatPointsNodeTests, test)
{
    RGLTestUsePointsNodeHelper<TestPointIsHit> helper;
    rgl_node_t compactNode = helper.simulateEmptyPointCloudOutputNode();

    fields.pop_back();
    fields.pop_back();
    fields.push_back(PADDING_32);
	fields.push_back(PADDING_32);
	fields.push_back(PADDING_8);
    fields.push_back(IS_HIT_I32);

	struct FormatStructExtended
	{
		Field<PADDING_32>::type padding;
		Field<PADDING_32>::type padding2;
		Field<PADDING_8>::type padding3;
        Field<IS_HIT_I32>::type isHit;
	} formatStructEx;

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));

    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));

    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

    int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, 0);
	EXPECT_EQ(outSizeOf,sizeof(formatStructEx));
}

TEST_F(FormatPointsNodeTests, from_graph_case_test)
{
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, 100).toRGL();
    prepareSceneWithCube(cubePoseTf);
    rayTf.push_back(Mat3x4f::identity().toRGL());

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));

    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNode));

    ASSERT_RGL_SUCCESS(rgl_graph_run(useRaysNode));

    int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, rayTf.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData(outCount);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], 0, EPSILON_F);
		EXPECT_NEAR(formatData[i].xyz[1], 0, EPSILON_F);
		EXPECT_NEAR(formatData[i].xyz[2], 99, EPSILON_F);
	}

    // ------------------------------------------------------------------------

    fields.push_back(DISTANCE_F32);  // Add distance field
	// fields.push_back(PADDING_32);  // Align to 8 bytes

	struct FormatStructExtended : public  FormatStruct
	{
		Field<DISTANCE_F32>::type distance;
		// Field<PADDING_32>::type padding2;  // Align to 8 bytes
	} formatStructEx;

	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	outCount = -1;  // reset variables
	outSizeOf = -1;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
	EXPECT_EQ(outCount, rayTf.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStructEx));

	std::vector<FormatStructExtended> formatDataEx{(size_t)outCount};
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, formatDataEx.data()));

	for (int i = 0; i < formatDataEx.size(); ++i) {
		EXPECT_NEAR(formatDataEx[i].xyz[0], 0, EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].xyz[1], 0, EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].xyz[2], 99, EPSILON_F);
		EXPECT_NEAR(formatDataEx[i].distance, 99, EPSILON_F);
	}
}

//Empty fields