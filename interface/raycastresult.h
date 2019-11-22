#pragma once
#include <vector>
#include <string>
#include <chrono>
#include "lidarpoint.h"

typedef struct RaycastResult
{
    chrono::time_point acquisitionTime;
    std::string lidarID;
    std::vector<LidarPoint> points;
} RaycastResult;

typedef std::vector<RaycastResult> RaycastResults;
