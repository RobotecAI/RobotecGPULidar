using System;
using System.Runtime.InteropServices;

namespace GPULidarRaycaster
{

// Would need a typedef here since 1) structs can't be derived in C# 2) "using" is local
// using Point3f = System.Numerics.Vector3;
// using Vector3f = System.Numerics.Vector3;
// using Vector2f = System.Numerics.Vector2;

[StructLayout(LayoutKind.Sequential)]
public struct Vector3f {
  public float x;
  public float y;
  public float z;
}

[StructLayout(LayoutKind.Sequential)]
public struct Vector2f {
  public float x;
  public float y;
}

[StructLayout(LayoutKind.Sequential)]
public struct Point3f {
  public float x;
  public float y;
  public float z;
}

[StructLayout(LayoutKind.Sequential)]
public struct Vector3i {
  public Int32 x;
  public Int32 y;
  public Int32 z;
}

}  // namespace GPULidarRaycaster
