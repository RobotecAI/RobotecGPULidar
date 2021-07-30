#pragma once
#include "points.h"
#include <string>
#include <vector>

struct LidarSource {
    std::string unique_id;
    Point source; //global coordinates
    std::vector<Point> directions;
    float range;
};
