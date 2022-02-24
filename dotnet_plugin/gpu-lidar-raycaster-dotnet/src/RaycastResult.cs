using System;
using System.Numerics;

namespace GPULidarRaycaster
{

public struct RaycastResults {
  public String lidar_id;
  public int pointCount;
  public Point3f[] xyz;
  public byte[] rosPCL12;
  public byte[] rosPCL24;
  public byte[] rosPCL48;
}

} // namespace GPULidarRaycaster
