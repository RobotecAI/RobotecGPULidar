using System;
using System.Numerics;

namespace GPULidarRaycaster
{

struct LidarSource {
  String id;
  Point3f source;
  Point3f[] directions;
  float range;
}

} // namespace GPULidarRaycaster
