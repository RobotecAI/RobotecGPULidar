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
#include "Lidar.h"

// our helper library for window handling
#include "glfWindow/GLFWindow.h"
#include <GL/gl.h>

#include <cmath>
#include <fstream>

//#define LIDAR_2D

std::string pointsFileName = "points.xyz";

void savePointsToFile(std::vector<float> &points);


long long current_timestamp()
{
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return milliseconds;
}


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
                 int samplingInitialHeight,
                 bool is2D,
                 float range)
        : GLFCameraWindow(title, camera.from, camera.at, camera.up, worldScale),
          lidarRend(model, range), sample(model), lidar(lidarInitialSource, lidarInitialDirection, lidarInitialWidth, 
          lidarInitialHeight, samplingInitialWidth, samplingInitialHeight, is2D, range)
    {
        sample.setCamera(camera);
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
        lidarRend.resize(lidar.rays.size()/6);
        lidarRend.render(lidar.rays);
        lidarRend.downloadPoints(points);
        savePointsToFile(points);
        sample.resizeLidar(points.size()/6);
        sample.render(points);
        
//        printf("%lld\n", current_timestamp()-begin);
//        begin = current_timestamp();
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
        switch(key)
        {
        case 235: // forward
            lidar.moveX(10.f);
            break;
        case 264: // back
            lidar.moveX(-10.f);
            break;
        case 262: // right
            lidar.moveZ(10.f);
            break;
        case 263: // left
            lidar.moveZ(-10.f);
            break;
        case 65: // a
            lidar.rotateY(-0.1f);
            break;
        case 68: // d
            lidar.rotateY(0.1f);
            break;
        case 83: // s
            lidar.rotateZ(-0.1f);
            break;
        case 87: // w
            lidar.rotateZ(0.1f);
            break;
        }
        
        /*
        // move source
        if (key == 265) // forward
        {
            lidar.moveX(10.f);
        }
        if (key == 264) // back
        {
            lidar.moveX(-10.f);
        }
        if (key == 263) // left
        {
            lidar.moveZ(-10.f);
        }
        if (key == 262) // right
        {
            lidar.moveZ(10.f);
        }
            
        // rotate source
        
        if (key == 65) // a
        {
            lidar.rotateY(-0.1f);
        }
        if (key == 68) // d
        {
            lidar.rotateY(0.1f);
        }
        if (key == 83) // s
        {
            lidar.rotateZ(-0.1f);
        }
        if (key == 87) // w
        {
            lidar.rotateZ(0.1f);
        }
        */
    }


    vec2i                 fbSize;
    GLuint                fbTexture {0};
    LidarRenderer         lidarRend;
    SampleRenderer        sample;
    Lidar                 lidar;
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
        Camera camera = { /*from*/vec3f(-4000.07f, 450.f, 0.f),
 //                         /* at */model->bounds.center()-vec3f(0,400,0),
                          /* at */vec3f(1,0.06,0),
                          /* up */vec3f(0.f,1.f,0.f) };
        // something approximating the scale of the world, so the
        // camera knows how much to move for any given user interaction:
        const float worldScale = length(model->bounds.span());
        
        // lidar setting
        vec3f lidarInitialSource = vec3f(-4000.f, 450.f, 0.f);
        vec3f lidarInitialDirection = vec3f(1.f, 0.f, 0.f);
        float lidarInitialWidth = 240*M_PI/180.f; // angle in radians
        float lidarInitialHeight = 30*M_PI/180.f; // angle in radians
        int samplingInitialWidth = 30;
        int samplingInitialHeight = 10;
        
        /*
        // this values we need to compare
        float lidarInitialWidth = 240*M_PI/180.f; // angle in radians
        float lidarInitialHeight = 30*M_PI/180.f; // angle in radians
        int samplingInitialWidth = 1149;
        int samplingInitialHeight = 240;
        */
        
        bool is2D = false;
        float range = 2000.f; // 40m * 50

        SampleWindow *window = new SampleWindow("Optix lidar",
                                                model, camera, worldScale,
                                                lidarInitialSource, lidarInitialDirection,
                                                lidarInitialWidth, lidarInitialHeight, samplingInitialWidth,
                                                samplingInitialHeight, is2D, range);
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
    file << points.size()/6 << "\n\n";
    
    for (int i = 0; i < points.size()/6; ++i)
    {
        file << points[6*i+0] << " " << points[6*i+1] << " " << points[6*i+2] << " ";

        const int r = int(255.99f*points[6*i+3]);
        const int g = int(255.99f*points[6*i+3]);
        const int b = int(255.99f*points[6*i+3]);

        // convert to 32-bit rgba value (alpha set to 0xff)
        const uint32_t rgba = 0xff000000 | (r<<0) | (g<<8) | (b<<16);
        file << r << '\n';
//        file << (points[6*i+3] + points[6*i+4] + points[6*i+5])/3 << '\n';
    }
    file.close();
}
