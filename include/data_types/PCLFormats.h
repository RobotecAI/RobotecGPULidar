#pragma once

#include <cstddef>
#include <cstdint>

// These types describe formats of ROS2 sensor_msgs.msg.PointCloud2.data binary array.
// If you are reading this code in H2 2022 or later, it means something went terribly wrong.

// Current output from the Robotec GPU Lidar
struct PCL12
{
    float x;
    float y;
    float z;
};
static_assert(sizeof(PCL12) == 12);
static_assert(offsetof(PCL12, x) == 0);
static_assert(offsetof(PCL12, y) == 4);
static_assert(offsetof(PCL12, z) == 8);

// Format requested by TierIV
struct PCL24
{
    float x;
    float y;
    float z;
    uint32_t _pad0;
    float intensity;
    uint16_t ring;
};
static_assert(sizeof(PCL24) == 24);
static_assert(offsetof(PCL24, x) == 0);
static_assert(offsetof(PCL24, y) == 4);
static_assert(offsetof(PCL24, z) == 8);
static_assert(offsetof(PCL24, intensity) == 16);
static_assert(offsetof(PCL24, ring) == 20);

// Format requested by TierIV
struct PCL48
{
    float x;
    float y;
    float z;
    uint32_t _pad0;
    float intensity;
    uint16_t ring;
    float azimuth;
    float distance;
    uint8_t return_type;
    double time_stamp;
};
static_assert(sizeof(PCL48) == 48);
static_assert(offsetof(PCL48, x) == 0);
static_assert(offsetof(PCL48, y) == 4);
static_assert(offsetof(PCL48, z) == 8);
static_assert(offsetof(PCL48, intensity) == 16);
static_assert(offsetof(PCL48, ring) == 20);
static_assert(offsetof(PCL48, azimuth) == 24);
static_assert(offsetof(PCL48, distance) == 28);
static_assert(offsetof(PCL48, return_type) == 32);
static_assert(offsetof(PCL48, time_stamp) == 40);
