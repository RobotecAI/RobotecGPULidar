#pragma once
#include "PointTypes.h"
#include <string>
#include <vector>

struct LidarSource {
    LidarSource(const char* id, Point3f sourcePoint, float range, int directionsCount, Point3f* directions)
        : unique_id(id)
        , source(sourcePoint)
        , directions(directions, directions + directionsCount)
        , range(range)
    {
    }

    std::string unique_id;
    Point3f source; //global coordinates
    std::vector<Point3f> directions;
    float range;
};
