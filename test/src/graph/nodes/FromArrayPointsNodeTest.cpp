#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>
#include <RGLFields.hpp>


class FromArrayPointsNodeTest : public RGLAutoCleanupTest
{
protected:
};


TEST_F(FromArrayPointsNodeTest, UsePointsFromArray)
{
    rgl_node_t usePoints=nullptr, transform=nullptr, format=nullptr;

    struct PointStruct
    {
        Field<XYZ_F32>::type xyz;
        Field<PADDING_32>::type padding;
        Field<IS_HIT_I32>::type isHit;
        Field<INTENSITY_F32>::type intensity;
    } pointStruct;

    std::vector<rgl_field_t> pointFields = {
        XYZ_F32,
        PADDING_32,
        IS_HIT_I32,
        INTENSITY_F32
    };

    std::vector<PointStruct> inPoints;
    inPoints.push_back(PointStruct { .xyz = {1, 2, 3}, .isHit = 1, .intensity = 100 } );
    inPoints.push_back(PointStruct { .xyz = {4, 5, 6}, .isHit = 1, .intensity = 200 } );

    rgl_mat3x4f xyzTf = Mat3x4f::TRS({1, 1, 1}).toRGL();

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePoints, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
    EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transform, &xyzTf));
    EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, pointFields.data(), pointFields.size()));

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePoints, transform));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transform, format));

    EXPECT_RGL_SUCCESS(rgl_graph_run(usePoints));

    int32_t outCount, outSizeOf;
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    EXPECT_EQ(outCount, inPoints.size());
    EXPECT_EQ(outSizeOf, sizeof(pointStruct));

    std::vector<PointStruct> outPoints{(size_t)outCount};
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, outPoints.data()));

    std::vector<PointStruct> expectedPoints;
    expectedPoints.push_back(PointStruct { .xyz = {2, 3, 4}, .isHit = 1, .intensity = 100 } );
    expectedPoints.push_back(PointStruct { .xyz = {5, 6, 7}, .isHit = 1, .intensity = 200 } );

    auto expectNearPoints = [](PointStruct p1, PointStruct p2) {
        EXPECT_NEAR(p1.xyz[0], p2.xyz[0], 1e-6);
        EXPECT_NEAR(p1.xyz[1], p2.xyz[1], 1e-6);
        EXPECT_NEAR(p1.xyz[2], p2.xyz[2], 1e-6);
        EXPECT_EQ(p1.isHit, p2.isHit);
        EXPECT_NEAR(p1.intensity, p2.intensity, 1e-6);
    };

    expectNearPoints(outPoints[0], expectedPoints[0]);
    expectNearPoints(outPoints[1], expectedPoints[1]);
}
