using System;
using System.Numerics;

namespace GPULidarRaycaster
{

public struct LidarSource {
  public String source_id;
  public Point3f source_pos;
  public Point3f[] directions;
  public float range;
}

} // namespace GPULidarRaycaster
