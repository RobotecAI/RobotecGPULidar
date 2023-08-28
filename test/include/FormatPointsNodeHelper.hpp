#pragma once

#include <RGLFields.hpp>
#include <random>
#include <scenes.hpp>

constexpr int MULTIPLE_FORMATS_COUNT = 15;

static std::random_device randomDevice;
static auto randomSeed = randomDevice();
static std::mt19937 randomGenerator { randomSeed };

struct FormatPointsNodeHelper {
    rgl_node_t usePointsNode = nullptr;
    rgl_node_t compactPointsNode = nullptr;
    rgl_node_t yieldPointsNode = nullptr;
    std::vector<rgl_node_t> formatNodes;

    FormatPointsNodeHelper()
        : formatNodes(MULTIPLE_FORMATS_COUNT, nullptr)
    {
    }

    struct PointWithoutPadding {
        Field<XYZ_F32>::type xyz;
        Field<IS_HIT_I32>::type isHit;
        Field<DISTANCE_F32>::type distance;

        static std::vector<rgl_field_t> getPointFields() { return { XYZ_F32, IS_HIT_I32, DISTANCE_F32 }; }
    };

    static std::vector<PointWithoutPadding> generatePointsWithoutPaddings(int pointsCount)
    {
        std::vector<PointWithoutPadding> points;
        points.reserve(pointsCount);
        for (int i = 0; i < pointsCount; ++i) {
            auto currentPoint = PointWithoutPadding {
                .xyz = { i, i + 1.1, i + 2.2 },
                .isHit = i % 2,
                .distance = static_cast<float>(sqrt(pow(i, 2) + pow(i + 1, 2) + pow(i + 2, 2)))
            };
            points.push_back(currentPoint);
        }
        return points;
    }

    struct Point1 {
        Field<XYZ_F32>::type xyz;
        Field<PADDING_32>::type padding;
        Field<IS_HIT_I32>::type isHit;
        Field<DISTANCE_F32>::type distance;

        static std::vector<rgl_field_t> getPointFields() { return { XYZ_F32, PADDING_32, IS_HIT_I32, DISTANCE_F32 }; }
    };

    struct Point2 {
        Field<XYZ_F32>::type xyz;
        Field<PADDING_32>::type padding;
        Field<IS_HIT_I32>::type isHit;
        Field<PADDING_32>::type padding2;
        Field<DISTANCE_F32>::type distance;

        static std::vector<rgl_field_t> getPointFields() { return { XYZ_F32, PADDING_32, IS_HIT_I32, PADDING_32, DISTANCE_F32 }; }
    };

    struct Point3 {
        Field<XYZ_F32>::type xyz;
        Field<PADDING_32>::type padding;
        Field<IS_HIT_I32>::type isHit;
        Field<PADDING_32>::type padding2;
        Field<DISTANCE_F32>::type distance;
        Field<PADDING_32>::type padding3;

        static std::vector<rgl_field_t> getPointFields() { return { XYZ_F32, PADDING_32, IS_HIT_I32, PADDING_32, DISTANCE_F32, PADDING_32 }; }
    };

    struct Point4 {
        Field<PADDING_32>::type padding;
        Field<XYZ_F32>::type xyz;
        Field<PADDING_32>::type padding2;
        Field<IS_HIT_I32>::type isHit;
        Field<PADDING_32>::type padding3;
        Field<DISTANCE_F32>::type distance;
        Field<PADDING_32>::type padding4;

        static std::vector<rgl_field_t> getPointFields() { return { PADDING_32, XYZ_F32, PADDING_32, IS_HIT_I32, PADDING_32, DISTANCE_F32, PADDING_32 }; }
    };

    struct isHitAlonePoint {
        Field<IS_HIT_I32>::type isHit;

        static std::vector<rgl_field_t> getPointFields() { return { IS_HIT_I32 }; }
    };

    struct xyzAlonePoint {
        Field<XYZ_F32>::type xyz;

        static std::vector<rgl_field_t> getPointFields() { return { XYZ_F32 }; }
    };
};