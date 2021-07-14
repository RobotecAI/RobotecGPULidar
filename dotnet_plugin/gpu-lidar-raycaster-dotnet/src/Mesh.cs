using System;
using System.Numerics;

namespace GPULidarRaycaster
{

public class Mesh {
  public String id;

  // Whether coordinates in Mesh are in global reference frame
  public bool is_global = true;

  // If not global, this transform will be applied to vertices 
  public float[] transform = new float[12] { 1, 0, 0, 0,
                                             0, 1, 0, 0,
                                             0, 0, 1, 0 };
  public Vector3f[] vertices;
  public Vector3f[] normals;
  public Vector2f[] texture_coordinates;
  public Vector3i[] indices;
}

} // namespace GPULidarRaycaster
