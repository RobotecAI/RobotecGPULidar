#include <memory>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <unordered_map>

#include "optix_lidar.h"
#include "points.h"
#include "lidar_source.h"
#include "raycast_result.h"
#include "visibility_control.h"

using namespace gdt;
static std::string last_gpu_library_error = ""; // no support for multithreading

#define LIDAR_GPU_TRY_CATCH( call )                                                               \
{                                                                                                 \
  try {                                                                                           \
    call;                                                                                         \
  }                                                                                               \
  catch (std::runtime_error & err) {                                                              \
    last_gpu_library_error = err.what();                                                          \
    fprintf(stderr, "Runtime exception %s", err.what());                                          \
    return GPULIDAR_ERROR;                                                                        \
  }                                                                                               \
}

extern "C" {

// First attempt - on any error the module is deemed unusable
enum GPULIDAR_RETURN_CODE {
  GPULIDAR_SUCCESS = 0,
  GPULIDAR_ERROR
};

GPU_LIDAR_RAYCASTER_C_EXPORT
OptiXLidar * Internal_CreateNativeRaycaster()
{
  return new OptiXLidar();
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_DestroyNativeRaycaster(void * obj)
{
  delete (OptiXLidar *)obj;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
const char * Internal_GetLastError()
{ // Return pointer to memory of last_gpu_library_error. Interpreted as null-terminated string
  return last_gpu_library_error.c_str();
}

// TODO - optimize this POC
GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_AddOrUpdateMesh(void * obj, char * id, vec3f * vertices, vec3f * normals,
  vec2f * texture_coordinates, vec3i * indices, int indices_size, int size)
{
  auto *ol = (OptiXLidar *)obj;

  //Constructor could already use the pointers
  auto tm = std::make_shared<TriangleMesh>();

  std::vector<vec3f> v(vertices, vertices+size);
  tm->vertex = v;

  std::vector<vec3f> n(normals, normals+size);
  tm->normal = n;

  std::vector<vec2f> tc(texture_coordinates, texture_coordinates+size);
  tm->texcoord = tc;

  std::vector<vec3i> ind(indices, indices+indices_size);
  tm->index = ind;

  std::string mesh_id(id);
  tm->mesh_id = id;

  LIDAR_GPU_TRY_CATCH(ol->add_or_update_mesh(tm));
  return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_Raycast(void * obj, char * source_id, Point source_pos, Point * directions, int directions_count,
  float range)
{
  auto *ol = (OptiXLidar *)obj;
  LidarSource ls;
  ls.unique_id = std::string(source_id);
  ls.source = source_pos;
  std::vector<Point> d(directions, directions+directions_count);
  ls.directions = d;
  ls.range = range;
  LIDAR_GPU_TRY_CATCH(ol->raycast(ls));
  return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_GetPoints(void * obj, LidarPoint ** results, int * results_count)
{
  auto *ol = (OptiXLidar *)obj;

  LIDAR_GPU_TRY_CATCH(ol->get_all_points());

  const RaycastResults& rr = ol->last_results();
  if (rr.size() == 0) {
    results_count = 0;
    return GPULIDAR_SUCCESS;
  }
  auto & r = rr[0]; // TODO

  *results_count = r.points.size();
  *results = (LidarPoint*)r.points.data();
  return GPULIDAR_SUCCESS;
}

} // extern "C"
