
#include "visibility_control.h"
#include <memory>

#include "Model.h"

extern "C" {

class ModelLoader
{
public:
  ModelLoader() : model_(nullptr) {};
  ~ModelLoader() {};

  void load_obj(const std::string & path) {
    model_.reset(loadOBJ(path));
  }

  size_t get_number_of_meshes() {
    return model_ ? model_->meshes.size() : 0;
  }

  TriangleMesh * get_triangle_mesh(const uint32_t &mesh_index) {
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
    return model_ ? model_->meshes[mesh_index] : nullptr;
  }

  private:
  std::shared_ptr<Model> model_;

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
TriangleMesh * Internal_GetTriangleMesh(void * obj, int mesh_id,
  vec3f ** vertices, vec3f ** normals, vec2f ** texture_coordinates, vec3i ** indices,
  int * vertices_size, int * normals_size, int * texutres_size, int * indices_size)
{
  auto *ml = (ModelLoader *)obj;
  auto mesh = ml->get_triangle_mesh(mesh_id);
  *vertices = mesh->vertex.data();
  *vertices_size = mesh->vertex.size();
  *normals = mesh->normal.data();
  *normals_size = mesh->normal.size();
  *texture_coordinates = mesh->texcoord.data();
  *texutres_size = mesh->texcoord.size();
  *indices = mesh->index.data();
  *indices_size = mesh->index.size();
  return mesh;
}
}
