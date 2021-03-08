using System;
using System.Numerics;

namespace GPULidarRaycaster
{

public struct Mesh {
  public String id;
  public Vector3f[] vertices;
  public Vector3f[] normals;
  public Vector2f[] texture_coordinates;
  public Vector3i[] indices;
}

} // namespace GPULidarRaycaster
