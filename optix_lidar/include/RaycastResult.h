#pragma once
#include "PointTypes.h"
#include <chrono>
#include <string>
#include <vector>

typedef struct RaycastResult {
    std::chrono::system_clock::time_point acquisitionTime;
    std::string lidarID;
    std::vector<LidarPoint> points;
} RaycastResult;

typedef std::vector<RaycastResult> RaycastResults;
