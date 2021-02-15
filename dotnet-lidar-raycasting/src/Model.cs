using System
using System.Numerics.Vectors

using Vector3f = System.Numerics.Vector3;
using Vector2f = System.Numerics.Vector2;

namespace GPULidarRaycaster
{

struct Vector3i {
  Int32 x;
  Int32 y;
  Int32 z;
}

struct Model {
  String id;
  Vector3f[] vertices;
  Vector3f[] normals;
  Vector2f[] texture_coordinates;
  Vector3i[] index;
}

struct Texture {
  Uint32[] pixel;
}

} // namespace GPULidarRaycaster
