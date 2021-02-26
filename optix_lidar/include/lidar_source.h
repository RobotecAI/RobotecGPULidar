#pragma once
#include <string>
#include <vector>
#include "points.h"

struct LidarSource
{
    std::string unique_id;
    Point source; //global coordinates
    std::vector<Point> directions;
    float range;
};
