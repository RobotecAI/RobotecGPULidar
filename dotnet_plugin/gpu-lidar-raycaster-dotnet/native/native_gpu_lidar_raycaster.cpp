#include <memory>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "lidarsource.h"
#include "raycastresult.h"
#include "Model.h"
#include "visibility_control.h"

extern "C" {

class NativeRaycaster
{
public:
  NativeRaycaster() {};
  ~NativeRaycaster() {};
  void addModel(std::shared_ptr<TriangleMesh> model)
  {
    models_.push_back(model);
    printf("Size of model vertices: %lu\n", model->vertex.size());
  }

private:
  std::vector<std::shared_ptr<TriangleMesh>> models_;
};

GPU_LIDAR_RAYCASTER_C_EXPORT
NativeRaycaster * Internal_CreateNativeRaycaster() {
  return new NativeRaycaster();
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_DestroyNativeRaycaster(void * obj) {
  delete (NativeRaycaster *)obj;
}

// TODO - optimize this POC
GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_AddModel(void * obj, vec3f * vertices, vec3f * normals,
  vec2f * texture_coordinates, vec3i * indices, int size) {
  auto tm = std::make_shared<TriangleMesh>();

  std::vector<vec3f> v(vertices, vertices+size);
  tm->vertex = v;
  printf("size is %lu\n", v.size());
  auto *nr = (NativeRaycaster *)obj;
  nr->addModel(tm);
}


/*
GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_UpdateModel(void * obj, void * model) {
  // Implement
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_Raycast(void * obj, void * source, void ** results) {
  // Implement
}
*/

}
