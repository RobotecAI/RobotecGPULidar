using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace GPULidarRaycaster
{

public class Raycaster : IDisposable
{
  private IntPtr m_NativeRaycaster = IntPtr.Zero;
  public Raycaster() {
    m_NativeRaycaster = Internal_CreateNativeRaycaster();
  }

  ~Raycaster() {
    Dispose(false);
  }

  public void Dispose()
  {
    Dispose(true);
  }

  private void Dispose(bool disposing)
  {
    if (!disposed)
    {
      if(disposing)
      {
          // Dispose managed resources.
      }

      Destroy();
      disposed = true;
    }
  }

  public void AddModel(Model model) {
    NativeHandleCheck();
    if (Internal_HasModel(m_NativeRaycaster, model.id)) {
      return;
    }
    Internal_AddModel(m_NativeRaycaster, model.id, model.vertices, model.normals,
      model.texture_coordinates, model.indices, model.vertices.Length);
  }

  public void UpdateModel(Model model) {
    NativeHandleCheck();
    if (!Internal_HasModel(m_NativeRaycaster, model.id)) {
      return;
    }
    Internal_UpdateModel(m_NativeRaycaster, model.id, model.vertices, model.normals,
      model.texture_coordinates, model.indices, model.vertices.Length);
  }

  public void Raycast(LidarSource source, ref RaycastResults res) {
    NativeHandleCheck();
    Internal_Raycast(m_NativeRaycaster, source.source_id, source.source_pos, source.directions,
      source.directions.Length, source.range);

    // Get points right away - this could also be done in a different time
    IntPtr results_raw = new IntPtr();
    int results_count = 0;
    Internal_GetPoints(m_NativeRaycaster, source.source_id, ref results_raw, ref results_count);

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

  private void NativeHandleCheck() {
    if (m_NativeRaycaster == IntPtr.Zero) {
      throw new Exception("Native raycaster object not created or destroyed already!");
    }
  }

  private void Destroy() {
    if (m_NativeRaycaster != IntPtr.Zero) {
      Internal_DestroyNativeRaycaster(m_NativeRaycaster);
      m_NativeRaycaster = IntPtr.Zero;
    }
  }

  //TODO - wrap & optimize
  [DllImport("libnative_gpu_lidar_raycaster.so", CallingConvention = CallingConvention.Cdecl)]
  private static extern IntPtr Internal_CreateNativeRaycaster();
  [DllImport("libnative_gpu_lidar_raycaster.so", CallingConvention = CallingConvention.Cdecl)]
  private static extern void Internal_DestroyNativeRaycaster(IntPtr obj);

  [DllImport("libnative_gpu_lidar_raycaster.so", CallingConvention = CallingConvention.Cdecl)]
  private static extern bool Internal_HasModel(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id);
  [DllImport("libnative_gpu_lidar_raycaster.so", CallingConvention = CallingConvention.Cdecl)]
  private static extern void Internal_AddModel(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id, [In] Vector3f[] vertices,
    [In] Vector3f[] normals, [In] Vector2f[] texture_coordinates, [In] Vector3i[] indices,
    int size);
  [DllImport("libnative_gpu_lidar_raycaster.so", CallingConvention = CallingConvention.Cdecl)]
  private static extern void Internal_UpdateModel(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id, [In] Vector3f[] vertices,
    [In] Vector3f[] normals, [In] Vector2f[] texture_coordinates, [In] Vector3i[] indices,
    int size);

  [DllImport("gpu-lidar-raycaster", CallingConvention = CallingConvention.Cdecl)]
  private static extern void Internal_UpdateModel(IntPtr obj, IntPtr model);

  [DllImport("gpu-lidar-raycaster", CallingConvention = CallingConvention.Cdecl)]
  private static extern void Internal_Raycast(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string source_id, [In] Point3f source_pos,
    [In] Point3f[] directions, [In] int directions_count, [In] float range);

  [DllImport("gpu-lidar-raycaster", CallingConvention = CallingConvention.Cdecl)]
  private static extern void Internal_GetPoints(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string source_id,
    ref IntPtr results, ref int results_count);

  private bool disposed;
}

} // namespace GPULidarRaycaster
