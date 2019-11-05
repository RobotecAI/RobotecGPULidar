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

void uploadRays(std::vector<float> &rays);

struct SampleWindow : public GLFCameraWindow
{
    SampleWindow(const std::string &title,
                 const Model *model,
                 const Camera &camera,
                 const float worldScale,
                 std::vector<float> &rays)
        : GLFCameraWindow(title, camera.from, camera.at, camera.up, worldScale),
          lidar(model), sample(model), lidarRays(rays)
    {
        sample.setCamera(camera);
    }
    
    virtual void render() override
    {
        if (cameraFrame.modified) {
            sample.setCamera(Camera{cameraFrame.get_from(),
                                    cameraFrame.get_at(),
                                    cameraFrame.get_up() });
            cameraFrame.modified = false;
        }
        std::vector<float> points;
//printf("\nlidar\n");
        lidar.resize(lidarRays.size()/6);
        lidar.render(lidarRays);
        lidar.downloadPoints(points);
//printf("\npo lidarze\n");
        sample.resizeLidar(points.size()/3);
        sample.render(points);
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
//        lidar.resize(newSize);
        fbSize = newSize;
        sample.resize(newSize);
        pixels.resize(newSize.x*newSize.y);
    }

    vec2i                 fbSize;
    GLuint                fbTexture {0};
    LidarRenderer         lidar;
    SampleRenderer        sample;
    std::vector<float>    lidarRays;
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

        std::vector<float> rays;
        uploadRays(rays);

        SampleWindow *window = new SampleWindow("Optix lidar",
                                                model, camera, worldScale, rays);
        window->run();
      
    } catch (std::runtime_error& e) {
      std::cout << GDT_TERMINAL_RED << "FATAL ERROR: " << e.what()
                << GDT_TERMINAL_DEFAULT << std::endl;
      exit(1);
    }
    return 0;
}
  

void uploadRays(std::vector<float> &rays)
{
    rays.push_back(0.f);
    rays.push_back(2000.f);
    rays.push_back(0.f);
    
    rays.push_back(0.f);
    rays.push_back(-1.f);
    rays.push_back(0.f);

/*
    rays.push_back(500.f);
    rays.push_back(2000.f);
    rays.push_back(500.f);
    
    rays.push_back(0.f);
    rays.push_back(-1.f);
    rays.push_back(0.f);
    
    rays.push_back(4000.f);
    rays.push_back(2000.f);
    rays.push_back(-16.f);
    
    rays.push_back(0.f);
    rays.push_back(-1.f);
    rays.push_back(0.f);
    
    rays.push_back(1228.f);
    rays.push_back(2000.f);
    rays.push_back(-237.f);
    
    rays.push_back(0.f);
    rays.push_back(-1.f);
    rays.push_back(0500.f);
/*
    // line from (-2, 0, -2) to (2, 0, 2)
    vec3f raySource = vec3f(-250.f, 150.f, 0.f);
    int sampling = 64;
    float wide = 300;
    for (int i = 0; i < sampling; ++i)
    {
        float x = i*wide/(float)sampling - wide/2;
        vec3f dest = vec3f(0.f, 150.f, x);
        vec3f dir = dest - raySource;
        
        rays.push_back(raySource.x);
        rays.push_back(raySource.y);
        rays.push_back(raySource.z);
        rays.push_back(dir.x);
        rays.push_back(dir.y);
        rays.push_back(dir.z);
    }

*/
/*
    // for now in one plane
    vec3f lidarSource = vec3f(0.f, 2000.f, 0.f);
    vec3f lidarDirection = vec3f(0.f, -1.f, 0.f);
    float lidarCone = 1.f; // in radians
    float lidarAngle = std::atan(lidarDirection.z/lidarDirection.y);
    int sampling = 64;
    
    for (int i = 0; i < sampling; ++i)
    {
        float y = std::tan(lidarCone*i/(float)sampling - lidarCone/2 + lidarAngle)*lidarDirection.z;
        float z = std::tan(y) > 0 ? lidarDirection.z : -lidarDirection.z;
        vec3f dir = vec3f(0.f, y, z);
        
        rays.push_back(lidarSource.x);
        rays.push_back(lidarSource.y);
        rays.push_back(lidarSource.z);
        rays.push_back(dir.x);
        rays.push_back(dir.y);
        rays.push_back(dir.z);
//printf("%f %f %f, %f %f %f\n", lidarSource.x, lidarSource.y, lidarSource.z, dir.x, dir.y, dir.z);
    }
*/
}
