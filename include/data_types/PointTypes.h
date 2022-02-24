#pragma once

template <typename CoordinateType>
struct Point3D {
    CoordinateType x;
    CoordinateType y;
    CoordinateType z;
} __attribute__((packed));

// template <typename CoordinateType>
// struct Lidar4DPoint {
//     CoordinateType x;
//     CoordinateType y;
//     CoordinateType z;
//     CoordinateType i;
// } __attribute__((packed));

typedef Point3D<float> Point3f;
// typedef Lidar4DPoint<float> LidarPoint;
