#pragma once
#include "PointTypes.h"
#include <chrono>
#include <string>
#include <vector>

typedef struct RaycastResult {
    std::vector<LidarPoint> points;
} RaycastResult;

typedef std::vector<RaycastResult> RaycastResults;
