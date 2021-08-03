#pragma once
#include "PointTypes.h"
#include <string>
#include <vector>

struct LidarSource {
    LidarSource(const char* id, Point sourcePoint, float range, int directionsCount, Point* directions)
    : unique_id(id)
    , source(sourcePoint)
    , directions(directions, directions + directionsCount)
    , range(range)
    {
    }

    std::string unique_id;
    Point source; //global coordinates
    std::vector<Point> directions;
    float range;
};
