#pragma once

template <typename CoordinateType>
struct Point3D {
    CoordinateType x;
    CoordinateType y;
    CoordinateType z;
} __attribute__((packed));

typedef Point3D<float> Point3f;

