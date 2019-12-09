#include "Lidar.h"

Lidar::Lidar(vec3f source, vec3f direction, float width, float height, int resolutionWidth, int resolutionHeight, float range)
    : range(range), source(source),  direction(direction), width(width), height(height),
      resolutionWidth(resolutionWidth), resolutionHeight(resolutionHeight)
{
    generateRays();
}


void Lidar::setResolution(int width, int height)
{
    resolutionWidth = width;
    resolutionHeight = height;
}

void Lidar::setAngles(int widthAngle, int heightAngle)
{
    width = widthAngle;
    height = heightAngle;
}

void Lidar::moveX(float step)
{
    source.x += step;
    generateRays();
}

void Lidar::moveZ(float step)
{
    source.z += step;
    generateRays();
}

void Lidar::rotateY(float angle)
{
    vec3f dirP = vec3f(direction);
    dirP.x = direction.x*cos(angle) - direction.z*sin(angle);
    dirP.z = direction.x*sin(angle) + direction.z*cos(angle);
    
    direction = dirP;
    generateRays();
}

void Lidar::rotateZ(float angle)
{
    vec3f dirP = vec3f(direction);
    dirP.x = direction.x*cos(angle) - direction.y*sin(angle);
    dirP.y = direction.x*sin(angle) + direction.y*cos(angle);
    
    direction = dirP;
    generateRays();
}

void Lidar::generateRays()
{
    rays.clear();
    
    for (int i = 0; i < resolutionWidth; ++i)
    {
        for (int j = 0; j < resolutionHeight; ++j)
        {
            float angle1 = i*width/(float)resolutionWidth - width/2;
            float angle2 = j*height/((float)resolutionHeight) - height/2;
            
            // rotation on z axis
            vec3f dirP = vec3f(direction);
            dirP.x = direction.x*cos(angle2) - direction.y*sin(angle2);
            dirP.y = direction.x*sin(angle2) + direction.y*cos(angle2);
            
            // rotation on y axis
            vec3f dir = vec3f(dirP);
            dir.x = dirP.x*cos(angle1) - dirP.z*sin(angle1);
            dir.z = dirP.x*sin(angle1) + dirP.z*cos(angle1);
            
            rays.push_back(source.x);
            rays.push_back(source.y);
            rays.push_back(source.z);
            rays.push_back(dir.x);
            rays.push_back(dir.y);
            rays.push_back(dir.z);
//printf("%f %f %f, %f %f %f\n", lidarSource.x, lidarSource.y, lidarSource.z, dir.x, dir.y, dir.z);
        }
    }
}
