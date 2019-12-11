#pragma once
#include <memory>

template<typename CoordinateType>
struct Lidar4DPoint
{
    CoordinateType x;
    CoordinateType y;
    CoordinateType z;
    CoordinateType i;
};

typedef Lidar4DPoint<float> LidarPoint;
