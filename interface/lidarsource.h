#pragma once
#include <string>
#include "primitives.h"

struct LidarSource
{
    std::string unique_id;
    Point source; //global coordinates
    std::vector<Point> directions;
    float range;
};
