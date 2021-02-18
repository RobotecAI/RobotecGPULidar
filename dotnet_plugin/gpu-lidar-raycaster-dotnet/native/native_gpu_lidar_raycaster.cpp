#include <memory>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <unordered_map>

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

  void add_or_update_model(char * id, vec3f * vertices, vec3f * normals,
    vec2f * texture_coordinates, vec3i * indices, int size)
  {
    //Constructor could already use the pointers
    auto tm = std::make_shared<TriangleMesh>();

    std::vector<vec3f> v(vertices, vertices+size);
    tm->vertex = v;

    std::vector<vec3f> n(normals, normals+size);
    tm->normal = n;

    std::vector<vec2f> tc(texture_coordinates, texture_coordinates+size);
    tm->texcoord = tc;

    std::vector<vec3i> ind(indices, indices+size);
    tm->index = ind;

    std::string model_id(id);
    models_[model_id] = tm;
  }

  bool has_model(const std::string & id)
  {
    return models_.find(id) != models_.end();
  }

  void raycast(const LidarSource & lidar_source)
  {
    // TODO - reuse memory from last raycast
    auto result = std::make_shared<RaycastResult>();

    // TODO - do the actual raycasting

    // TODO - remove this adding of dummy test points
    LidarPoint pseudoPoint{1, 2, 3, 5};
    result->points.push_back(pseudoPoint);
    result->points.push_back(pseudoPoint);
    result->points.push_back(pseudoPoint);

    std::vector<LidarPoint> points;

    results_[lidar_source.unique_id] = result;
  }

  std::shared_ptr<RaycastResult> get_points_for_source(const std::string & source_id) const
  {
    if (results_.find(source_id) == results_.end()) {
      return std::make_shared<RaycastResult>();
    }
    return results_.at(source_id);
  }

private:
  std::unordered_map<std::string, std::shared_ptr<TriangleMesh>> models_;
  std::unordered_map<std::string, std::shared_ptr<RaycastResult>> results_;
};

GPU_LIDAR_RAYCASTER_C_EXPORT
NativeRaycaster * Internal_CreateNativeRaycaster()
{
  return new NativeRaycaster();
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_DestroyNativeRaycaster(void * obj)
{
  delete (NativeRaycaster *)obj;
}

// TODO - optimize this POC
GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_AddModel(void * obj, char * id, vec3f * vertices, vec3f * normals,
  vec2f * texture_coordinates, vec3i * indices, int size)
{
  auto *nr = (NativeRaycaster *)obj;
  if (nr->has_model(id))
    return;

  nr->add_or_update_model(id, vertices, normals, texture_coordinates, indices, size);
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_UpdateModel(void * obj, char * id, vec3f * vertices, vec3f * normals,
  vec2f * texture_coordinates, vec3i * indices, int size)
{
  auto *nr = (NativeRaycaster *)obj;
  if (!nr->has_model(id))
    return;

  nr->add_or_update_model(id, vertices, normals, texture_coordinates, indices, size);
}

GPU_LIDAR_RAYCASTER_C_EXPORT
bool Internal_HasModel(void * obj, char * id)
{
  auto *nr = (NativeRaycaster *)obj;
  std::string model_id(id);
  return nr->has_model(model_id);
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_Raycast(void * obj, char * source_id, Point source_pos, Point * directions, int directions_count,
  float range)
{
  auto *nr = (NativeRaycaster *)obj;
  LidarSource ls;
  ls.unique_id = std::string(source_id);
  ls.source = source_pos;
  std::vector<Point> d(directions, directions+directions_count);
  ls.directions = d;
  ls.range = range;
  nr->raycast(ls);

}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_GetPoints(void * obj, char * source_id, LidarPoint ** results, int * results_count)
{
  auto *nr = (NativeRaycaster *)obj;
  auto r = nr->get_points_for_source(std::string(source_id));
  *results_count = r->points.size();
  *results = r->points.data();
}

} // extern "C"
