#include <utils.hpp>
#include <UsePointsNodeHelper.hpp>
#include <FormatPointsNodeHelper.hpp>

class FormatPointsNodeTests : public RGLTest, public FormatPointsNodeHelper {};

TEST_F(FormatPointsNodeTests, invalid_argument_node)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");
}

TEST_F(FormatPointsNodeTests, invalid_argument_fields)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNodes.at(0), nullptr, 0), "fields != nullptr");
}

TEST_F(FormatPointsNodeTests, invalid_argument_field_count)
{
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), 0), "field_count > 0");
}

TEST_F(FormatPointsNodeTests, valid_arguments)
{
    EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), formats.at(0).size()));
    ASSERT_THAT(formatNodes.at(0), testing::NotNull());

    // If (*formatNodes.at(0)) != nullptr
    EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), formats.at(0).size()));
}

TEST_F(FormatPointsNodeTests, error_when_invalid_field_request)
{
    prepareSceneWithCube();
    rayTf.push_back(Mat3x4f::identity().toRGL());

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));

    formats.at(0).push_back(TIME_STAMP_F64);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), formats.at(0).size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNodes[0]));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(formatNodes.at(0)), "cannot get time from scene");

    formats.at(0).pop_back();
    formats.at(0).push_back(RING_ID_U16);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), formats.at(0).size()));
    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(formatNodes.at(0)), "cannot get ring ids");

    formats.at(0).pop_back();
    formats.at(0).push_back(RGL_FIELD_DYNAMIC_FORMAT);
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), formats.at(0).size()), "cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");

    formats.at(0).pop_back();
    formats.at(0).push_back((rgl_field_t)12);
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), formats.at(0).data(), formats.at(0).size()));
    EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_run(formatNodes.at(0)), "unknown RGL field");
}

TEST_F(FormatPointsNodeTests, format_when_empty_point_cloud_input)
{
    RGLTestUsePointsNodeHelper<TestPointIsHit> helper;
    rgl_node_t compactNode = helper.simulateEmptyPointCloudOutputNode();

    auto fields = helper.generator->getPointFields();

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(0), fields.data(), fields.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, formatNodes.at(0)));
    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(0)));

    int32_t outCount, outSizeOf;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, 0);
	EXPECT_EQ(outSizeOf, sizeof(Point));
}

TEST_F(FormatPointsNodeTests, should_properly_align_output_size)
{
    prepareSceneWithCube();
    rayTf.push_back(Mat3x4f::identity().toRGL());

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));
    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(2), formats.at(2).data(), formats.at(2).size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNodes.at(2)));
    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(2)));

    int32_t outCount, outSizeOf;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(2), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
	EXPECT_EQ(outCount, rayTf.size());

    size_t expectedOutSize = getPointSize(formats.at(2));
	EXPECT_EQ(outSizeOf, expectedOutSize);
}

TEST_F(FormatPointsNodeTests, format_node_results_data_should_be_the_same_as_from_yield_node)
{
    prepareSceneWithCube();
    rayTf.push_back(Mat3x4f::identity().toRGL());

    float maxAngle = atan(CUBE_HALF_EDGE / (CUBE_Z_DISTANCE - CUBE_HALF_EDGE));

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
    // fields = { XYZ_F32, IS_HIT_I32, DISTANCE_F32 };

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));
    ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldPointsNode, formats.at(0).data(), formats.at(0).size()));
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

    ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(4), formats.at(4).data(), formats.at(4).size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(raytraceNode, yieldPointsNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNodes.at(4)));
    ASSERT_RGL_SUCCESS(rgl_graph_run(formatNodes.at(4)));

    int32_t outCount, outSizeOf;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(4), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point4> outFormatPoints(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(4), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));
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
    prepareSceneWithCube();
    rayTf.push_back(Mat3x4f::identity().toRGL());

    float maxAngle = atan(CUBE_HALF_EDGE / (CUBE_Z_DISTANCE - CUBE_HALF_EDGE));

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

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));   
    for(auto& formatNode : formatNodes) {
        ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNode, formats.at(0).data(), formats.at(0).size()));
    }
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNodes.at(0)));
    for(size_t i = 1; i < formatNodes.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatNodes.at(i - 1), formatNodes.at(i)));
    }

    ASSERT_RGL_SUCCESS(rgl_graph_run(useRaysNode));

    //Compare results from first and last format node
    int32_t outCount, outSizeOf;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point> outFormatPoints(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));
    
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(formatNodes.size() - 1), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point> outFormatPointsLast(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(formatNodes.size() - 1), RGL_FIELD_DYNAMIC_FORMAT, outFormatPointsLast.data()));

    ASSERT_EQ(outFormatPoints.size(), outFormatPointsLast.size());
    for(size_t i = 0; i < outFormatPoints.size(); ++i) {
        EXPECT_EQ(outFormatPoints[i].xyz[0], outFormatPointsLast[i].xyz[0]);
        EXPECT_EQ(outFormatPoints[i].xyz[1], outFormatPointsLast[i].xyz[1]);
        EXPECT_EQ(outFormatPoints[i].xyz[2], outFormatPointsLast[i].xyz[2]);
        EXPECT_EQ(outFormatPoints[i].isHit, outFormatPointsLast[i].isHit);
        EXPECT_EQ(outFormatPoints[i].distance, outFormatPointsLast[i].distance);
    }
}

TEST_F(FormatPointsNodeTests, multiple_changing_format_should_not_change_data)
{
    prepareSceneWithCube();
    rayTf.push_back(Mat3x4f::identity().toRGL());

    float maxAngle = atan(CUBE_HALF_EDGE / (CUBE_Z_DISTANCE - CUBE_HALF_EDGE));

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

    for (size_t i = 0 ; i < formats.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatNodes.at(i), formats.at(i).data(), formats.at(i).size()));
    }

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rayTf.data(), rayTf.size()));
    ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, RAYTRACE_DEPTH));  
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNodes.at(0)));
    for(size_t i = 1; i < formatNodes.size(); ++i) {
        ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatNodes.at(i - 1), formatNodes.at(i)));
    }

    ASSERT_RGL_SUCCESS(rgl_graph_run(useRaysNode));

    int32_t outCount, outSizeOf;
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point> outFormatPoints(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(0), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints.data()));

    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(1), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point1> outFormatPoints2(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(1), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints2.data()));
    
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(2), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point2> outFormatPoints3(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(2), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints3.data()));

    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(3), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point3> outFormatPoints4(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(3), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints4.data()));

    ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(formatNodes.at(4), RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    std::vector<Point4> outFormatPoints5(outCount);
    ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(formatNodes.at(4), RGL_FIELD_DYNAMIC_FORMAT, outFormatPoints5.data()));


    ASSERT_EQ(outFormatPoints.size(), outFormatPoints2.size());
    ASSERT_EQ(outFormatPoints.size(), outFormatPoints3.size());
    ASSERT_EQ(outFormatPoints.size(), outFormatPoints4.size());
    ASSERT_EQ(outFormatPoints.size(), outFormatPoints5.size());

    for(size_t i = 0; i < 5 ; ++i) {
        EXPECT_EQ(outFormatPoints[i].xyz[0], outFormatPoints2[i].xyz[0]);
        EXPECT_EQ(outFormatPoints[i].xyz[1], outFormatPoints2[i].xyz[1]);
        EXPECT_EQ(outFormatPoints[i].xyz[2], outFormatPoints2[i].xyz[2]);
        EXPECT_EQ(outFormatPoints[i].isHit, outFormatPoints2[i].isHit);
        EXPECT_EQ(outFormatPoints[i].distance, outFormatPoints2[i].distance);
        
        EXPECT_EQ(outFormatPoints[i].xyz[0], outFormatPoints3[i].xyz[0]);
        EXPECT_EQ(outFormatPoints[i].xyz[1], outFormatPoints3[i].xyz[1]);
        EXPECT_EQ(outFormatPoints[i].xyz[2], outFormatPoints3[i].xyz[2]);
        EXPECT_EQ(outFormatPoints[i].isHit, outFormatPoints3[i].isHit);
        EXPECT_EQ(outFormatPoints[i].distance, outFormatPoints3[i].distance);

        EXPECT_EQ(outFormatPoints[i].xyz[0], outFormatPoints4[i].xyz[0]);
        EXPECT_EQ(outFormatPoints[i].xyz[1], outFormatPoints4[i].xyz[1]);
        EXPECT_EQ(outFormatPoints[i].xyz[2], outFormatPoints4[i].xyz[2]);
        EXPECT_EQ(outFormatPoints[i].isHit, outFormatPoints4[i].isHit);
        EXPECT_EQ(outFormatPoints[i].distance, outFormatPoints4[i].distance);

        EXPECT_EQ(outFormatPoints[i].xyz[0], outFormatPoints5[i].xyz[0]);
        EXPECT_EQ(outFormatPoints[i].xyz[1], outFormatPoints5[i].xyz[1]);
        EXPECT_EQ(outFormatPoints[i].xyz[2], outFormatPoints5[i].xyz[2]);
        EXPECT_EQ(outFormatPoints[i].isHit, outFormatPoints5[i].isHit);
        EXPECT_EQ(outFormatPoints[i].distance, outFormatPoints5[i].distance);
    }
}
