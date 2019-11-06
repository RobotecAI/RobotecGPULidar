// ======================================================================== //
// Copyright 2018-2019 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "SampleRenderer.h"
#include "LidarRenderer.h"

// our helper library for window handling
#include "glfWindow/GLFWindow.h"
#include <GL/gl.h>

#include <cmath>
#include <fstream>

//#define LIDAR_2D

std::string pointsFileName = "points.xyz";

void calculateRays(std::vector<float> &rays);
void savePointsToFile(std::vector<float> &points);

/*
long long current_timestamp()
{
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return milliseconds;
}
*/

struct SampleWindow : public GLFCameraWindow
{
    SampleWindow(const std::string &title,
                 const Model *model,
                 const Camera &camera,
                 const float worldScale,
                 vec3f lidarInitialSource,
                 vec3f lidarInitialDirection,
                 float lidarInitialWidth,
                 float lidarInitialHeight,
                 int samplingInitialWidth,
                 int samplingInitialHeight)
        : GLFCameraWindow(title, camera.from, camera.at, camera.up, worldScale),
          lidar(model), sample(model), lidarSource(lidarInitialSource),lidarDirection(lidarInitialDirection),
          lidarWidth(lidarInitialWidth), lidarHeight(lidarInitialHeight), 
          samplingWidth(samplingInitialWidth), samplingHeight(samplingInitialHeight)
    {
        sample.setCamera(camera);
        calculateRays();
    }
    
    virtual void render() override
    {
        // for calculating frame rendering time
//        static long long begin = current_timestamp();

        if (cameraFrame.modified) {
            sample.setCamera(Camera{cameraFrame.get_from(),
                                    cameraFrame.get_at(),
                                    cameraFrame.get_up() });
            cameraFrame.modified = false;
        }
        std::vector<float> points;
        lidar.resize(lidarRays.size()/6);
        lidar.render(lidarRays);
        lidar.downloadPoints(points);
        savePointsToFile(points);
        sample.resizeLidar(points.size()/3);
        sample.render(points);
        
        //printf("%lld\n", current_timestamp()-begin);
        //begin = current_timestamp();
    }
    
    virtual void draw() override
    {
        sample.downloadPixels(pixels.data());
        if (fbTexture == 0)
            glGenTextures(1, &fbTexture);
      
        glBindTexture(GL_TEXTURE_2D, fbTexture);
        GLenum texFormat = GL_RGBA;
        GLenum texelType = GL_UNSIGNED_BYTE;
        glTexImage2D(GL_TEXTURE_2D, 0, texFormat, fbSize.x, fbSize.y, 0, GL_RGBA,
                     texelType, pixels.data());

        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, fbTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glDisable(GL_DEPTH_TEST);

        glViewport(0, 0, fbSize.x, fbSize.y);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.f, (float)fbSize.x, 0.f, (float)fbSize.y, -1.f, 1.f);

        glBegin(GL_QUADS);
        {
            glTexCoord2f(0.f, 0.f);
            glVertex3f(0.f, 0.f, 0.f);

            glTexCoord2f(0.f, 1.f);
            glVertex3f(0.f, (float)fbSize.y, 0.f);

            glTexCoord2f(1.f, 1.f);
            glVertex3f((float)fbSize.x, (float)fbSize.y, 0.f);

            glTexCoord2f(1.f, 0.f);
            glVertex3f((float)fbSize.x, 0.f, 0.f);
        }
        glEnd();
    }
    
    virtual void resize(const vec2i &newSize) 
    {
        fbSize = newSize;
        sample.resize(newSize);
        pixels.resize(newSize.x*newSize.y);
    }
    
    virtual void key(int key, int mods)
    {
//        printf("%d %d\n", key, mods);
        
        // move source
        if (key == 265) // forward
        {
            lidarSource.x += 10;
        }
        if (key == 264) // back
        {
            lidarSource.x -= 10;
        }
        if (key == 263) // left
        {
            lidarSource.z -= 10;
        }
        if (key == 262) // right
        {
            lidarSource.z += 10;
        }
            
        // rotate source
        
        if (key == 65) // a
        {
            float angle = -0.1;
            vec3f dirP = vec3f(lidarDirection);
            dirP.x = lidarDirection.x*cos(angle) - lidarDirection.z*sin(angle);
            dirP.z = lidarDirection.x*sin(angle) + lidarDirection.z*cos(angle);
            
            lidarDirection = dirP;
        }
        if (key == 68) // d
        {
            float angle = 0.1;
            vec3f dirP = vec3f(lidarDirection);
            dirP.x = lidarDirection.x*cos(angle) - lidarDirection.z*sin(angle);
            dirP.z = lidarDirection.x*sin(angle) + lidarDirection.z*cos(angle);
            
            lidarDirection = dirP;
        }
        if (key == 83) // s
        {
            float angle = -0.1;
            vec3f dirP = vec3f(lidarDirection);
            dirP.x = lidarDirection.x*cos(angle) - lidarDirection.y*sin(angle);
            dirP.y = lidarDirection.x*sin(angle) + lidarDirection.y*cos(angle);
            
            lidarDirection = dirP;
        }
        if (key == 87) // w
        {
            float angle = 0.1;
            vec3f dirP = vec3f(lidarDirection);
            dirP.x = lidarDirection.x*cos(angle) - lidarDirection.y*sin(angle);
            dirP.y = lidarDirection.x*sin(angle) + lidarDirection.y*cos(angle);
            
            lidarDirection = dirP;
        }
        
        calculateRays();
    }

    void calculateRays()
    {
        lidarRays.clear();
        
#ifdef LIDAR_2D
        
        for (int i = 0; i < samplingWidth; ++i)
        {
            float angle = i*lidarWidth/(float)samplingWidth - lidarWidth/2;
            
            // rotation on y axis
            vec3f dir = vec3f(lidarDirection);
            dir.x = lidarDirection.x*cos(angle) - lidarDirection.z*sin(angle);
            dir.z = lidarDirection.x*sin(angle) + lidarDirection.z*cos(angle);
            
            lidarRays.push_back(lidarSource.x);
            lidarRays.push_back(lidarSource.y);
            lidarRays.push_back(lidarSource.z);
            lidarRays.push_back(dir.x);
            lidarRays.push_back(dir.y);
            lidarRays.push_back(dir.z);
//printf("%f %f %f, %f %f %f\n", lidarSource.x, lidarSource.y, lidarSource.z, dir.x, dir.y, dir.z);
        }
#else
        for (int i = 0; i < samplingWidth; ++i)
        {
            for (int j = 0; j < samplingHeight; ++j)
            {
                float angle1 = i*lidarWidth/(float)samplingWidth - lidarWidth/2;
                float angle2 = j*lidarHeight/((float)samplingHeight) - lidarHeight/2;
                
                // rotation on y axis
                vec3f dirP = vec3f(lidarDirection);
                dirP.x = lidarDirection.x*cos(angle1) - lidarDirection.z*sin(angle1);
                dirP.z = lidarDirection.x*sin(angle1) + lidarDirection.z*cos(angle1);
                
                // rotation on z axis
                vec3f dir = vec3f(dirP);
                dir.x = dirP.x*cos(angle2) - dirP.y*sin(angle2);
                dir.y = dirP.x*sin(angle2) + dirP.y*cos(angle2);
                
                lidarRays.push_back(lidarSource.x);
                lidarRays.push_back(lidarSource.y);
                lidarRays.push_back(lidarSource.z);
                lidarRays.push_back(dir.x);
                lidarRays.push_back(dir.y);
                lidarRays.push_back(dir.z);
//printf("%f %f %f, %f %f %f\n", lidarSource.x, lidarSource.y, lidarSource.z, dir.x, dir.y, dir.z);
            }
        }
#endif
    }

    vec2i                 fbSize;
    GLuint                fbTexture {0};
    LidarRenderer         lidar;
    SampleRenderer        sample;
    std::vector<float>    lidarRays;
    vec3f                 lidarSource;
    vec3f                 lidarDirection;
    float                 lidarWidth; // angle in radians
    float                 lidarHeight; // angle in radians
    int                   samplingWidth;
    int                   samplingHeight;
    std::vector<uint32_t> pixels;
};
  
  
/*! main entry point to this example - initially optix, print hello
world, then exit */
extern "C" int main(int ac, char **av)
{
    try {
        Model *model = loadOBJ(
#ifdef _WIN32
        // on windows, visual studio creates _two_ levels of build dir
        // (x86/Release)
        "../models/tunnel.obj"
#else
        // on linux, common practice is to have ONE level of build dir
        // (say, <project>/build/)...
        "../models/tunnel.obj"
#endif
                             );
        Camera camera = { /*from*/vec3f(-1293.07f, 154.681f, -0.7304f),
                          /* at */model->bounds.center()-vec3f(0,400,0),
                          /* up */vec3f(0.f,1.f,0.f) };
        // something approximating the scale of the world, so the
        // camera knows how much to move for any given user interaction:
        const float worldScale = length(model->bounds.span());
        
        // lidar setting
        vec3f lidarInitialSource = vec3f(-4000.f, 450.f, 0.f);
        vec3f lidarInitialDirection = vec3f(1.f, 0.06f, 0.f);
        float lidarInitialWidth = 1.f; // angle in radians
        float lidarInitialHeight = 0.5f; // angle in radians
        int samplingInitialWidth = 30;
        int samplingInitialHeight = 10;

        SampleWindow *window = new SampleWindow("Optix lidar",
                                                model, camera, worldScale,
                                                lidarInitialSource, lidarInitialDirection,
                                                lidarInitialWidth, lidarInitialHeight, samplingInitialWidth, samplingInitialHeight);
        window->run();
    
    } catch (std::runtime_error& e) {
      std::cout << GDT_TERMINAL_RED << "FATAL ERROR: " << e.what()
                << GDT_TERMINAL_DEFAULT << std::endl;
      exit(1);
    }
    return 0;
}

void savePointsToFile(std::vector<float> &points)
{
    std::ofstream file(pointsFileName);
    file << points.size()/3 << "\n\n";
    
    for (int i = 0; i < points.size()/3; ++i)
    {
        file << points[3*i] << " " << points[3*i+1] << " " << points[3*i+2] << '\n';
    }
    file.close();
}
