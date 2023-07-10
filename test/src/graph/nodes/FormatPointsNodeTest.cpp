#include <utils.hpp>
#include <UsePointsNodeHelper.hpp>
#include <scenes.hpp>
#include <RGLFields.hpp>

static constexpr float RAYTRACE_DEPTH = 1000;
static constexpr float CUBE_HALF_EDGE = 1.0;
static constexpr int ANGLE_ITERATIONS = 100;

class FormatPointsNodeTests : public RGLTest {
protected:
    rgl_node_t formatNode, useRaysNode, raytraceNode;

    std::vector<rgl_field_t> fields;
    std::vector<rgl_mat3x4f> rayTf;

    rgl_mesh_t cubeMesh;
    rgl_entity_t cubeEntity;

    FormatPointsNodeTests() 
    {
        fields = {XYZ_F32, PADDING_32};
        formatNode = nullptr, useRaysNode = nullptr, raytraceNode = nullptr;
    }

    struct FormatStruct
    {
        ::Field<XYZ_F32>::type xyz;
        ::Field<PADDING_32>::type padding;

        FormatStruct() = default;
        FormatStruct(Field<XYZ_F32>::type xyz)
            : xyz(xyz){}
    } formatStruct;

    void prepareSceneWithCube(rgl_mat3x4f cubePoseTf = Mat3x4f::identity().toRGL())
    {
        cubeMesh = makeCubeMesh();
        EXPECT_RGL_SUCCESS(rgl_entity_create(&cubeEntity, nullptr, cubeMesh));
        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cubeEntity, &cubePoseTf));
    }
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

TEST_F(FormatPointsNodeTests, error_when_invalid_field_request)
{
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, 100).toRGL();
    prepareSceneWithCube(cubePoseTf);
    rayTf.push_back(Mat3x4f::identity().toRGL());

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));

    fields.push_back(TIME_STAMP_F64);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNode));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(formatNode), "cannot get time from scene");

    fields.pop_back();
    fields.push_back(RING_ID_U16);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(formatNode), "cannot get ring ids");

    fields.pop_back();
    fields.push_back(RGL_FIELD_DYNAMIC_FORMAT);
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNode, fields.data(), fields.size()), "cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");

    fields.pop_back();
    fields.push_back((rgl_field_t)12);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_run(formatNode), "unknown RGL field");
}

TEST_F(FormatPointsNodeTests, format_when_empty_point_cloud_input)
{
    RGLTestUsePointsNodeHelper<TestPointIsHit> helper;
    rgl_node_t compactNode = helper.simulateEmptyPointCloudOutputNode();

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

    int32_t outCount, outSizeOf;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, 0);
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));
}

TEST_F(FormatPointsNodeTests, should_properly_align_output_size)
{
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, 100).toRGL();
    prepareSceneWithCube(cubePoseTf);
    rayTf.push_back(Mat3x4f::identity().toRGL());

    fields.push_back(PADDING_32);
	fields.push_back(PADDING_8);

	struct FormatStructExtended : public FormatStruct
	{
		Field<PADDING_32>::type padding2;
		Field<PADDING_8>::type padding3;

        FormatStructExtended() = default;
        FormatStructExtended(Field<XYZ_F32>::type xyz)
            : FormatStruct(xyz) {}
	} formatStructEx;

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

    int32_t outCount, outSizeOf;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
	EXPECT_EQ(outCount, rayTf.size());

    ASSERT_NE(outSizeOf, sizeof(formatStruct));

    size_t expectedOutSize = getPointSize(fields);
    size_t expectedStructSize = expectedOutSize + (8 - (expectedOutSize % 8));
    EXPECT_EQ(sizeof(formatStructEx), expectedStructSize);
	EXPECT_EQ(outSizeOf, expectedOutSize);
}

TEST_F(FormatPointsNodeTests, format_node_results_data_should_be_the_same_as_from_yield_node)
{
    float cubeZDistance = 100.0f;
    const rgl_mat3x4f cubePoseTf = Mat3x4f::translation(0, 0, cubeZDistance).toRGL();
    prepareSceneWithCube(cubePoseTf);

    float maxAngle = atan(CUBE_HALF_EDGE / (cubeZDistance - CUBE_HALF_EDGE));

    // Generate angle and print the seed
    std::random_device rd;
    auto seed = rd();
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> dis(-maxAngle - 0.2, maxAngle + 0.2);
    std::cerr << "FormatPointsNodeTests various_angle seed: " << seed << std::endl;

    for (int i = 0; i < ANGLE_ITERATIONS; ++i) {
        float randomAngle = dis(gen);
        rayTf.push_back(Mat3x4f::rotationRad(RGL_AXIS_X, randomAngle).toRGL());
    }

    rgl_node_t yieldPointsNode = nullptr;
    fields = { XYZ_F32, IS_HIT_I32, DISTANCE_F32 };

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));
    ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldPointsNode, fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldPointsNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(yieldPointsNode));

    std::vector<::Field<XYZ_F32>::type> outPoints;
    std::vector<::Field<IS_HIT_I32>::type> outIsHits;
    std::vector<::Field<DISTANCE_F32>::type> outDistances;
    int32_t outDistancesCount, outDistancesSize, outPointsCount, outPointsSize, outIsHitsCount, outIsHitsSize;

    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, XYZ_F32, &outPointsCount, &outPointsSize));
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, IS_HIT_I32, &outIsHitsCount, &outIsHitsSize));
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, DISTANCE_F32, &outDistancesCount, &outDistancesSize));
    ASSERT_EQ(outPointsCount, outIsHitsCount);
    ASSERT_EQ(outPointsCount, outDistancesCount);

    outPoints.resize(outPointsCount);
    outIsHits.resize(outIsHitsCount);
    outDistances.resize(outDistancesCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, XYZ_F32, outPoints.data()));
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, IS_HIT_I32, outIsHits.data()));
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, DISTANCE_F32, outDistances.data()));

    struct Point {
        ::Field<XYZ_F32>::type xyz;
        ::Field<IS_HIT_I32>::type isHit;
        ::Field<DISTANCE_F32>::type distance;
        ::Field<PADDING_32>::type padding;
    };

    fields.push_back(PADDING_32);

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(raytraceNode, yieldPointsNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNode));
    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNode));

    int32_t outCount, outSizeOf;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point> outFormatPoints(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));
    ASSERT_EQ(outPointsCount, outCount);

    // Compare yield points output and format points output
    for(size_t i = 0; i < outCount ; ++i) {
        EXPECT_EQ(outPoints[i][0], outFormatPoints[i].xyz[0]);
        EXPECT_EQ(outPoints[i][1], outFormatPoints[i].xyz[1]);
        EXPECT_EQ(outPoints[i][2], outFormatPoints[i].xyz[2]);
        EXPECT_EQ(outIsHits[i], outFormatPoints[i].isHit);
        EXPECT_EQ(outDistances[i], outFormatPoints[i].distance);
    }
}

TEST_F(FormatPointsNodeTests, multiple_format_should_not_change_data)
{
    
}

TEST_F(FormatPointsNodeTests, multiple_changing_format_should_not_change_data)
{

}
