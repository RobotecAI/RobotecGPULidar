using System

namespace GPULidarRaycaster
{

public class Raycaster : IDisposable
{
  private IntPtr m_NativeRaycaster = IntPtr.Zero;
  public Raycaster() {
    m_NativeRaycaster = Internal_CreateNativeRaycaster();
  }

  ~Raycaster() {
    Dispose(false)();
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
    Internal_AddModel(m_NativeRaycaster, model);
  }

  public void UpdateModel(Model model) {
    NativeHandleCheck();
    Internal_UpdateModel(m_NativeRaycaster, model);
  }

  public void Raycast(const LidarSource source, ref RaycastResult result) {
    NativeHandleCheck();
    Internal_Raycast(m_NativeRaycaster, source, ref result);
  }

  private void NativeHandleCheck() {
    if (m_NativeRaycaster == IntPtr.Zero) {
      throw new Exception("Native raycaster object not created or destroyed already!");
    }
  }

  private void Destroy() {
    if (m_NativeRaycaster != IntPtr.Zero) {
      Internal_DestroyNativeRaycaster();
      m_NativeRaycaster = IntPtr.Zero;
    }
  }

  //TODO - wrap & optimize
  [DllImport("gpu-lidar-raycaster")]
  private static extern IntPtr Internal_CreateNativeRaycaster();
  [DllImport("gpu-lidar-raycaster")]
  private static extern Internal_DestroyNativeRaycaster(IntPtr obj);
  [DllImport("gpu-lidar-raycaster")]
  private static extern void Internal_AddModel(IntPtr obj, IntPtr model);
  [DllImport("gpu-lidar-raycaster")]
  private static extern void Internal_UpdateModel(IntPtr obj, IntPtr model);
  [DllImport("gpu-lidar-raycaster")]
  private static extern void Internal_Raycast(IntPtr obj, IntPtr source);

  private bool disposed;
}

} // namespace GPULidarRaycaster
