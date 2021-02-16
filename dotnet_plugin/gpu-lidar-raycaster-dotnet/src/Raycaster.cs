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
    Internal_AddModel(m_NativeRaycaster, model.vertices, model.normals,
      model.texture_coordinates, model.indices, model.vertices.Length);
  }

  /*
  public void UpdateModel(Model model) {
    NativeHandleCheck();
    Internal_UpdateModel(m_NativeRaycaster, model);
  }

  public void Raycast(const LidarSource source, ref RaycastResult result) {
    NativeHandleCheck();
    Internal_Raycast(m_NativeRaycaster, source, ref result);
  }
  */

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
  private static extern void Internal_AddModel(IntPtr obj, [In] Vector3f[] vertices,
    [In] Vector3f[] normals, [In] Vector2f[] texture_coordinates, [In] Vector3i[] indices,
    int size);
  /*
  [DllImport("gpu-lidar-raycaster"), CallingConvention = CallingConvention.Cdecl]
  private static extern void Internal_UpdateModel(IntPtr obj, IntPtr model);
  [DllImport("gpu-lidar-raycaster"), CallingConvention = CallingConvention.Cdecl]
  private static extern void Internal_Raycast(IntPtr obj, IntPtr source, ref IntPtr result);
  */

  private bool disposed;
}

} // namespace GPULidarRaycaster
