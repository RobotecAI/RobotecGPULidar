#include <rgl/api/core.h>
#include <graph/Node.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>
#include "utils.hpp"

struct RGLPointsTestHelper {

protected:
    std::vector<rgl_field_t> pointFields = {
        XYZ_F32,
        IS_HIT_I32,
        INTENSITY_F32
    };

    struct TestPointStruct {
        Field<XYZ_F32>::type xyz;
        Field<IS_HIT_I32>::type isHit;
        Field<INTENSITY_F32>::type intensity;
    };

    enum class HitPointDensity {
        HALF_HIT = 0,
        ALL_NON_HIT,
        ALL_HIT,
        RANDOM
    };

    rgl_node_t usePointsNode = nullptr;
    std::vector<TestPointStruct> inPoints;

    // TODO: What if we want a less regular density of hit points? To consider, change the function below.
    // TODO2: Make a function that creates random number of non-hit points
    std::vector<TestPointStruct> GenerateTestPointsArray(
        int count, std::optional<rgl_mat3x4f> transform = std::nullopt, std::optional<int> isHit = std::nullopt)
    {
        std::vector<TestPointStruct> points;
        for (int i = 0; i < count; ++i) {
            auto currentPoint = TestPointStruct {
                .xyz = { i, i + 1, i + 2 },
                .isHit = isHit.value_or(i % 2), // TODO Instead of passing isHit argument, pass HitPointDensinity
                .intensity = 100 };

            currentPoint.xyz = Mat3x4f::fromRGL(transform.value_or(identityTestTransform)) * currentPoint.xyz;
            points.emplace_back(currentPoint);
        }
        return points;
    }

    std::vector<TestPointStruct> GenerateTestNonHitPointsArray(int count, std::optional<rgl_mat3x4f> transform = std::nullopt)
    {
        return GenerateTestPointsArray(count, transform.value_or(identityTestTransform), 0);
    }

    std::vector<TestPointStruct> GenerateTestHitPointsArray(int count, std::optional<rgl_mat3x4f> transform = std::nullopt)
    {
        return GenerateTestPointsArray(count, transform.value_or(identityTestTransform), 1);
    }

    void CreateTestUsePointsNode(int pointsCount, HitPointDensity type = HitPointDensity::HALF_HIT)
    {
        switch (type) {
        case HitPointDensity::HALF_HIT:
            inPoints = GenerateTestPointsArray(pointsCount);
            break;
        case HitPointDensity::ALL_HIT:
            inPoints = GenerateTestHitPointsArray(pointsCount);
            break;
        case HitPointDensity::ALL_NON_HIT:
            inPoints = GenerateTestNonHitPointsArray(pointsCount);
            break;
        default:
            inPoints = GenerateTestPointsArray(pointsCount);
            break;
        }

        EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
        ASSERT_THAT(usePointsNode, testing::NotNull());
    }
};