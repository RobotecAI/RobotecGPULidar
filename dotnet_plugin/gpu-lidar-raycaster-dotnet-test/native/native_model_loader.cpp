
#include "visibility_control.h"
#include <memory>

#include "model.h"
#include "model_utils.h"

using namespace gdt;

extern "C" {

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
