#pragma once

#include <vector>
#include "simple_uid_generator.h"
#include "TransformMatrix.h"

struct TriangleMesh {
    TriangleMesh() { mesh_id = generate_simple_uid(); }
    TriangleMesh(const std::string& id)
            : mesh_id(id)
    {
    }

    bool is_global_coords = true;
    TransformMatrix transform;
    std::string mesh_id;
    std::vector<gdt::vec3f> vertex;
    std::vector<gdt::vec3f> normal;
    std::vector<gdt::vec2f> texcoord;
    std::vector<gdt::vec3i> index;

    // material data:
    gdt::vec3f diffuse;
    int diffuseTextureID { -1 };

    TriangleMesh(const TriangleMesh&) = delete; // non construction-copyable
    TriangleMesh& operator=(const TriangleMesh&) = delete; // non copyable
};