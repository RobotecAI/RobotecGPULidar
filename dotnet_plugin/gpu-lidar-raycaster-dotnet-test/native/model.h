#pragma once

#include <vector>

#include <gdt/math/vec.h>
#include <gdt/math/box.h>
#include <mutex>
#include <unordered_map>

std::string generate_simple_uid()
{
    static std::mutex mutex;
    static long int uid_int = 0;

    std::unique_lock<std::mutex> lock(mutex);
    std::string uid = std::to_string(uid_int);
    uid_int++;
    return uid;
}

struct TransformMatrix
{
  static const size_t transform_floats = 12;
  float matrix_flat[transform_floats] = {0};
} __attribute__((packed));

//indexed triangle mesh
struct TriangleMesh
{
  TriangleMesh() { mesh_id = generate_simple_uid(); }
  TriangleMesh(const std::string & id) : mesh_id(id) {}

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

  TriangleMesh(const TriangleMesh &) = delete; // non construction-copyable
  TriangleMesh & operator=(const TriangleMesh &) = delete; // non copyable
};

// TODO - non string id used for indexing
struct Texture
{
  Texture(int id) : texture_id(std::to_string(id)) {}

  std::string texture_id;
  ~Texture() { if (pixel) delete[] pixel; }

  uint32_t *pixel { nullptr };
  gdt::vec2i resolution { -1 };

  Texture(const Texture &) = delete; // non construction-copyable
  Texture & operator=(const Texture &) = delete; // non copyable
};

using MeshesMap = std::unordered_map<std::string, std::shared_ptr<TriangleMesh>>;
using TexturesMap = std::unordered_map<std::string, std::shared_ptr<Texture>>;
using Meshes = std::vector<std::shared_ptr<TriangleMesh>>;
using Textures = std::vector<std::shared_ptr<Texture>>;

struct Model
{
  MeshesMap meshes_map;
  TexturesMap textures_map;

  //! bounding box of all vertices in the model
  // TODO - this is a display feature, move away from this struct
  gdt::box3f bounds;

  // TODO
  bool changed { false };
  bool textures_changed { false };
  bool needs_rebuild { true };
};
