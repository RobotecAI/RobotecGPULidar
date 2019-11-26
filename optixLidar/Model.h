#pragma once

#include "gdt/math/AffineSpace.h"
#include <vector>

using namespace gdt;

  /*! a simple indexed triangle mesh that our sample renderer will
      render */
struct TriangleMesh {
    std::vector<vec3f> vertex;
    std::vector<vec3f> normal;
    std::vector<vec2f> texcoord;
    std::vector<vec3i> index;

    // material data:
    vec3f              diffuse;
    int                diffuseTextureID { -1 };
};

struct Texture {
    ~Texture()
    { if (pixel) delete[] pixel; }

    uint32_t *pixel      { nullptr };
    vec2i     resolution { -1 };
};

struct Model {
    ~Model()
    {
        for (auto mesh : meshes) delete mesh;
        for (auto texture : textures) delete texture;
    }

    std::vector<TriangleMesh *> meshes;
    std::vector<Texture *>      textures;
    //! bounding box of all vertices in the model
    box3f bounds;
    
    bool moved;
    bool big;
};

Model *loadOBJ(const std::string &objFile);
