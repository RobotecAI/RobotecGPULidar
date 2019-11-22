#pragma once
#include <string>
#include "primitives.h"

struct LidarSource
{
    std::string unique_id;
    Vector3f source; //global coordinates
    std::vector<Vector3f> directions;
    float range;
};
