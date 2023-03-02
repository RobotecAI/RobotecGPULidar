#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

struct PointStruct {
    Field<XYZ_F32>::type xyz;
    Field<PADDING_32>::type padding;
    Field<IS_HIT_I32>::type isHit;
    Field<INTENSITY_F32>::type intensity;
} pointStruct;

class FromArrayPointsNodeTest : public RGLAutoCleanupTestWithParam<int> {
protected:
    static std::vector<PointStruct> GeneratePointsArray(int count)
    {
        std::vector<PointStruct> points;
        for (int i = 0; i < count; ++i) {
            points.push_back(PointStruct { .xyz = { i, i + 1, i + 2 }, .isHit = i % 2, .intensity = 100 });
        }
        return points;
    }

    std::vector<rgl_field_t> pointFields = {
        XYZ_F32,
        PADDING_32,
        IS_HIT_I32,
        INTENSITY_F32
    };
};

INSTANTIATE_TEST_SUITE_P(
    MyGroup, FromArrayPointsNodeTest,
    testing::Values(1, 16, 65536),
    [](const auto& info) {
        return "pointsCount_" + std::to_string(info.param);
    });

TEST_P(FromArrayPointsNodeTest, invalid_arguments)
{
    int pointsCount = GetParam();
    auto inPoints = GeneratePointsArray(pointsCount);

    rgl_node_t usePointsNode = nullptr;

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), 0), "field_count > 0");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), nullptr, pointFields.size()), "fields != nullptr");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, inPoints.data(), 0, pointFields.data(), pointFields.size()), "points_count > 0");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(&usePointsNode, nullptr, inPoints.size(), pointFields.data(), pointFields.size()), "points != nullptr");
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_from_array(nullptr, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()), "node != nullptr");
}
TEST_P(FromArrayPointsNodeTest, valid_arguments)
{
    int pointsCount = GetParam();
    auto inPoints = GeneratePointsArray(pointsCount);

    rgl_node_t usePointsNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());
}

TEST_P(FromArrayPointsNodeTest, use_case)
{
    int pointsCount = GetParam();
    auto inPoints = GeneratePointsArray(pointsCount);

    rgl_node_t usePointsNode = nullptr;
    rgl_node_t formatNode = nullptr;

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());

    EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, pointFields.data(), pointFields.size()));
    ASSERT_THAT(formatNode, testing::NotNull());

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, formatNode));

    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    int32_t outCount, outSizeOf;
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));
    EXPECT_EQ(outCount, inPoints.size());
    EXPECT_EQ(outSizeOf, sizeof(pointStruct));

    std::vector<PointStruct> outPoints { (size_t)outCount };
    EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, outPoints.data()));

    std::vector<PointStruct> expectedPoints = GeneratePointsArray(pointsCount);

    auto expectNearPoints = [](PointStruct p1, PointStruct p2) {
        EXPECT_NEAR(p1.xyz[0], p2.xyz[0], 1e-6);
        EXPECT_NEAR(p1.xyz[1], p2.xyz[1], 1e-6);
        EXPECT_NEAR(p1.xyz[2], p2.xyz[2], 1e-6);
        EXPECT_EQ(p1.isHit, p2.isHit);
        EXPECT_NEAR(p1.intensity, p2.intensity, 1e-6);
    };

    for (int i = 0; i < pointsCount; ++i) {
        expectNearPoints(outPoints[i], expectedPoints[i]);
    }
}
