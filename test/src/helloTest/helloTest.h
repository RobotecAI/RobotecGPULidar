#ifndef ROBOTECGPULIDAR_HELLOTEST_H
#define ROBOTECGPULIDAR_HELLOTEST_H

struct Params
{
    uchar4* image;
    unsigned int image_width;
};

struct RayGenData
{
    float r,g,b;
};

#endif //ROBOTECGPULIDAR_HELLOTEST_H
