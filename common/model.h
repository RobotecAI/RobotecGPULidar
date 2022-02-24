#pragma once

#include <vector>

#include <gdt/math/vec.h>
#include <gdt/math/box.h>
#include <mutex>
#include <unordered_map>
#include "TransformMatrix.h"
#include "TriangleMesh.h"
#include "Texture.h"

std::string generate_simple_uid()
{
    static std::mutex mutex;
    static long int uid_int = 0;

    std::unique_lock<std::mutex> lock(mutex);
    std::string uid = std::to_string(uid_int);
    uid_int++;
    return uid;
}

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
