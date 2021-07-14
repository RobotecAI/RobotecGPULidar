
#include "visibility_control.h"
#include <memory>

#include "model.h"
#include "model_utils.h"

using namespace gdt;

extern "C" {

class ModelLoader
{
public:
  void load_obj(const std::string & path) {
    model_ = load_model_from_obj_file(path);
    if (model_) {
      meshes_.clear();
      for (auto mesh_kv : model_->meshes_map) {
        meshes_.push_back(mesh_kv.second);
      }
    }
  }

  size_t get_number_of_meshes() {
    return meshes_.size();
  }

  std::shared_ptr<TriangleMesh> get_triangle_mesh(const uint32_t & mesh_index) {
    // DEBUG
    /*
    if (model_) {
      for (auto v : model_->meshes[mesh_index]->vertex) {
        std::cout << "v: " << v << std::endl;
      }
      for (auto n : model_->meshes[mesh_index]->normal) {
        std::cout << "n: " << n << std::endl;
      }
      for (auto i : model_->meshes[mesh_index]->index) {
        std::cout << "i: " << i << std::endl;
      }
      for (auto t : model_->meshes[mesh_index]->texcoord) {
        std::cout << "t: " << t << std::endl;
      }

      std::cout << std::endl << std::endl;
    }
    */

    //

    if (meshes_.size() < mesh_index)
      return nullptr;

    return meshes_[mesh_index];
  }

  private:
    std::shared_ptr<Model> model_;
    Meshes meshes_; // For index-based access
};

MODEL_LOADER_C_EXPORT
ModelLoader * Internal_CreateNativeModelLoader()
{
  return new ModelLoader();
}

MODEL_LOADER_C_EXPORT
void Internal_DestroyNativeModelLoader(void * obj)
{
  delete (ModelLoader *)obj;
}

MODEL_LOADER_C_EXPORT
void Internal_LoadModel(void * obj, char * path)
{
  auto *ml = (ModelLoader *)obj;
  ml->load_obj(std::string(path));
}

MODEL_LOADER_C_EXPORT
int Internal_GetNumberOfMeshes(void * obj)
{
  auto *ml = (ModelLoader *)obj;
  return ml->get_number_of_meshes();
}

MODEL_LOADER_C_EXPORT
void Internal_GetTriangleMesh(void * obj, int mesh_index,
  vec3f ** vertices, vec3f ** normals, vec2f ** texture_coordinates, vec3i ** indices,
  int * vertices_size, int * normals_size, int * texutres_size, int * indices_size)
{
  auto *ml = (ModelLoader *)obj;
  auto mesh = ml->get_triangle_mesh(mesh_index);
  *vertices = mesh->vertex.data();
  *vertices_size = mesh->vertex.size();
  *normals = mesh->normal.data();
  *normals_size = mesh->normal.size();
  *texture_coordinates = mesh->texcoord.data();
  *texutres_size = mesh->texcoord.size();
  *indices = mesh->index.data();
  *indices_size = mesh->index.size();
}
}
