using System
using System.Numerics.Vectors

using Point3f = System.Numerics.Vector3;

namespace GPURaycaster
{

struct LidarSource {
  String id;
  Point3f source;
  Point3f[] directions;
  float range;
}

} // namespace GPULidarRaycaster
