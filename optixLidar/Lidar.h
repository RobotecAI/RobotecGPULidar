#pragma once

#include <vector>
#include "gdt/math/AffineSpace.h"

using namespace gdt;

struct Lidar
{
public:
    Lidar(vec3f source, vec3f direction, float width, float height, int resolutionWidth, int resolutionHeight, float range);
    
    // set lidar resolution
    void setResolution(int width, int height);
    void setAngles(int widthAngle, int heightAngle);
    
    void moveX(float step);
    void moveZ(float step);
    
    void rotateY(float angle);
    void rotateZ(float angle);
    
    std::vector<float> rays;
    float              range; //40m
    vec3f              source;
    
protected:
    void generateRays();
    
    vec3f              direction;
    float              width; // angle in radians
    float              height; // angle in radians
    int                resolutionWidth;
    int                resolutionHeight;
};
