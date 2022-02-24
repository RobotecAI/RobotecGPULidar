#pragma once
#include "PointTypes.h"
#include "TransformMatrix.h"
#include <string>
#include <vector>

struct LidarSource {
    LidarSource(const char* id, TransformMatrix lidarPose, TransformMatrix postRaycastTransform, TransformMatrix* rayPoses, int rayPoseCount, int* lidarArrayRingIds, int lidarArrayRingCount, float range)
        : unique_id(id)
        , lidarPose(lidarPose)
        , postRaycastTransform(postRaycastTransform)
        , rayPoses(rayPoses)
        , rayPoseCount(rayPoseCount)
        , lidarArrayRingIds(lidarArrayRingIds)
        , lidarArrayRingCount(lidarArrayRingCount)
        , range(range)
    {
    }

    LidarSource(const char* id, TransformMatrix lidarPose, TransformMatrix* rayPoses, int rayPoseCount, int* lidarArrayRingIds, int lidarArrayRingCount, float range)
    : unique_id(id)
    , lidarPose(lidarPose)
    , postRaycastTransform()
    , rayPoses(rayPoses)
    , rayPoseCount(rayPoseCount)
    , lidarArrayRingIds(lidarArrayRingIds)
    , lidarArrayRingCount(lidarArrayRingCount)
    , range(range)
    {
    }

    std::string unique_id;
    TransformMatrix lidarPose;
    TransformMatrix postRaycastTransform;
    TransformMatrix* rayPoses;
    int rayPoseCount;
    int* lidarArrayRingIds;
    int lidarArrayRingCount;

    float range;
};
