#pragma once

template<typename CoordinateType>
struct Point3D
{
    CoordinateType x;
    CoordinateType y;
    CoordinateType z;
};

typedef Point3D<float> Point;
