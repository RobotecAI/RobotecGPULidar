#include <rgl/api/core.h>
#include <graph/Node.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>
#include "utils.hpp"
#include <stdlib.h>

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
    int32_t randomNonHitCount = 0;

    std::vector<TestPointStruct> GenerateTestPointsArray(
            int count,
            rgl_mat3x4f transform = identityTestTransform,
            HitPointDensity hitPointDensity = HitPointDensity::HALF_HIT
            )
    {
        std::vector<TestPointStruct> points;
        randomNonHitCount = 0;

        for (int i = 0; i < count; ++i) {
            auto currentPoint = TestPointStruct {
                .xyz = { i, i + 1, i + 2 },
                .isHit = [&](){
                    switch(hitPointDensity){
                        case HitPointDensity::HALF_HIT:
                            return i % 2;
                        case HitPointDensity::ALL_NON_HIT:
                            return 0;
                        case HitPointDensity::ALL_HIT:
                            return 1;
                        case HitPointDensity::RANDOM:
                        {
                            int isHit = 0;
                            if(!isHit)
                                randomNonHitCount++;
                            return isHit;
                        }
                        default:
                            return i % 2;
                    }
                }(),
                .intensity = 100 };
            currentPoint.xyz = Mat3x4f::fromRGL(transform) * currentPoint.xyz;
            points.emplace_back(currentPoint);
        }
        return points;
    }

    void CreateTestUsePointsNode(
            int pointsCount,
            rgl_mat3x4f transform = identityTestTransform,
            HitPointDensity hitPointDensity = HitPointDensity::HALF_HIT)
    {
        inPoints = GenerateTestPointsArray(pointsCount, transform, hitPointDensity);

        EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&usePointsNode, inPoints.data(), inPoints.size(), pointFields.data(), pointFields.size()));
        ASSERT_THAT(usePointsNode, testing::NotNull());
    }

    std::vector<TestPointStruct> removeNonHit() { //TODO change name
        std::vector<TestPointStruct> hitPoints;
        for(auto point: inPoints) {
            if(point.isHit == 1) {
                hitPoints.push_back(point);
            }
        }
        return hitPoints;
    }
};