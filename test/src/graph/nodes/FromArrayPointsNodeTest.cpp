#include <RGLFields.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

struct PointStruct {
    Field<XYZ_F32>::type xyz;
    Field<IS_HIT_I32>::type isHit;
    Field<INTENSITY_F32>::type intensity;
};

class FromArrayPointsNodeTest : public RGLAutoCleanupTestWithParam<int> {
protected:
    static std::vector<PointStruct> GeneratePointsArray(int count)
    {
        std::vector<PointStruct> points;
        for (int i = 0; i < count; ++i) {
            points.emplace_back(PointStruct { .xyz = { i, i + 1, i + 2 }, .isHit = i % 2, .intensity = 100 });
        }
        return points;
    }

    std::vector<rgl_field_t> pointFields = {
        XYZ_F32,
        IS_HIT_I32,
        INTENSITY_F32
    };
};

INSTANTIATE_TEST_SUITE_P(
    MyGroup, FromArrayPointsNodeTest,
    testing::Values(1, 10, 100000),
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

    EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
    ASSERT_THAT(usePointsNode, testing::NotNull());

    EXPECT_RGL_SUCCESS(rgl_graph_run(usePointsNode));

    std::vector<PointStruct> expectedPoints = GeneratePointsArray(pointsCount);

    auto* expectedXYZ = new float[pointsCount * 3];
    for (int i = 0; i < pointsCount; ++i) {
        expectedXYZ[3 * i] = expectedPoints[i].xyz[0];
        expectedXYZ[(3 * i) + 1] = expectedPoints[i].xyz[1];
        expectedXYZ[(3 * i) + 2] = expectedPoints[i].xyz[2];
    }

    auto* expectedIsHit = new int[pointsCount];
    for (int i = 0; i < pointsCount; ++i) {
        expectedIsHit[i] = expectedPoints[i].isHit;
    }

    auto* expectedIntensity = new float[pointsCount];
    for (int i = 0; i < pointsCount; ++i) {
        expectedIntensity[i] = expectedPoints[i].intensity;
    }

    for (auto field : pointFields) {
        int32_t outCount, outSizeOf;
        EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(usePointsNode, field, &outCount, &outSizeOf));
        EXPECT_EQ(outCount, inPoints.size());
        EXPECT_EQ(outSizeOf, getFieldSize(field));

        void* outData = malloc(outCount * outSizeOf);
        EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(usePointsNode, field, outData));

        for (int i = 0; i < outCount; ++i) {
            if (field == XYZ_F32) {
                EXPECT_NEAR(expectedXYZ[3 * i], ((float*)outData)[3 * i], 1e-6);
                EXPECT_NEAR(expectedXYZ[(3 * i) + 1], ((float*)outData)[(3 * i) + 1], 1e-6);
                EXPECT_NEAR(expectedXYZ[(3 * i) + 2], ((float*)outData)[(3 * i) + 2], 1e-6);
            }
            if (field == IS_HIT_I32) {
                EXPECT_NEAR(expectedIsHit[i], ((int32_t*)outData)[i], 1e-6);
            }
            if (field == INTENSITY_F32) {
                EXPECT_NEAR(expectedIntensity[i], ((float*)outData)[i], 1e-6);
            }
        }
        free(outData);
    }
}