#pragma once

#include <type_traits>

// TODO: replace all occurences with Vec3f
struct Point3f {
    float x, y, z;
};
static_assert(sizeof(Point3f) == 3 * sizeof(float));
static_assert(std::is_trivially_copyable<Point3f>::value);

