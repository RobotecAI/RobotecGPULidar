#pragma once
#include <vector>
#include <string>
#include <chrono>
#include "lidarpoint.h"

typedef struct RaycastResult
{
    std::chrono::system_clock::time_point acquisitionTime;
    std::string lidarID;
    std::vector<LidarPoint> points;
} RaycastResult;

typedef std::vector<RaycastResult> RaycastResults;
