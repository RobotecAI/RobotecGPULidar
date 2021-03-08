using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace GPULidarRaycaster
{

public class Raycaster : IDisposable
{
  private IntPtr m_NativeRaycaster = IntPtr.Zero;
  public Raycaster()
  {
    m_NativeRaycaster = NativeMethods.Internal_CreateNativeRaycaster();
  }

  ~Raycaster()
  {
    Dispose(false);
  }

  public void Dispose()
  {
    Dispose(true);
  }

  private void Dispose(bool disposing)
  {
    if (!disposed) {
      if(disposing) {
        // Dispose managed resources.
      }
      Destroy();
      disposed = true;
    }
  }

  private void CheckError(int native_error_code)
  {
    if (native_error_code == 0)
      return;

    IntPtr errorPtr = NativeMethods.Internal_GetLastError();
    string error = Marshal.PtrToStringAnsi(errorPtr);
    throw new Exception("Native raycaster exception: [" + error + "]");
  }

  public void AddOrUpdateMesh(Mesh mesh)
  {
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_AddOrUpdateMesh(m_NativeRaycaster, mesh.id, mesh.vertices,
      mesh.normals, mesh.texture_coordinates, mesh.indices, mesh.indices.Length, mesh.vertices.Length));
  }

  public void Raycast(LidarSource source, ref RaycastResults res)
  {
    NativeHandleCheck();
    Console.WriteLine("Raycasting");
    CheckError(NativeMethods.Internal_Raycast(m_NativeRaycaster, source.source_id,
     source.source_pos, source.directions, source.directions.Length, source.range));

    // Get points right away - this could also be done in a different time
    IntPtr results_raw = new IntPtr();
    int results_count = 0;
    Console.WriteLine("Getting points.");
    CheckError(NativeMethods.Internal_GetPoints(m_NativeRaycaster, ref results_raw, ref results_count));

    res.lidar_id = source.source_id;
    res.points = new Point4f[results_count];
    if (results_count > 0) {
      int single_point_number_of_floats = Marshal.SizeOf(res.points[0]) / sizeof(float);
      int floats_total = results_count * single_point_number_of_floats;
      float[] flat = new float[floats_total];
      Marshal.Copy(results_raw, flat, 0, floats_total);
      for (int i = 0; i < results_count; ++i) {
        int flat_offset = i * single_point_number_of_floats;
        res.points[i].x = flat[flat_offset];
        res.points[i].y = flat[flat_offset + 1];
        res.points[i].z = flat[flat_offset + 2];
        res.points[i].i = flat[flat_offset + 3];
      }
    }
  }

  private void NativeHandleCheck()
  {
    if (m_NativeRaycaster == IntPtr.Zero) {
      throw new Exception("Native raycaster object not created or destroyed already!");
    }
  }

  private void Destroy()
  {
    if (m_NativeRaycaster != IntPtr.Zero) {
      NativeMethods.Internal_DestroyNativeRaycaster(m_NativeRaycaster);
      m_NativeRaycaster = IntPtr.Zero;
    }
  }

  private bool disposed;
}

} // namespace GPULidarRaycaster
