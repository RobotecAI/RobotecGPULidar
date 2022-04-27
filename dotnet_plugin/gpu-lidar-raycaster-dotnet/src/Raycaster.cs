using System;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Collections.Generic;

namespace GPULidarRaycaster
{

public class Raycaster : IDisposable
{
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

  public IntPtr CreateLidarContext(in LidarSource source)
  {
    NativeHandleCheck();
    IntPtr ctx = IntPtr.Zero;
    CheckError(NativeMethods.Internal_CreateLidarContext(
            IntPtr.Zero,
            out ctx,
            source.sourcePoses,
            source.sourcePoses.Length,
            source.lidarArrayRingIds,
            source.lidarArrayRingIds.Length
    ));
    return ctx;
  }

  public void DestroyLidarContext(IntPtr ctx)
  {
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_DestroyLidarContext(
            IntPtr.Zero,
            ctx
    ));
  }

  public void RaycastAsync(in LidarSource source, IntPtr lidarContext, double timestamp)
  {
    // At the moment timestamp is not yet propagated to the native side.
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_Raycast(
        IntPtr.Zero,
        lidarContext,
        source.source_id,
        source.lidarPose,
        source.postRaycastTransform,
        source.range
    ));
  }

  public void PrepareDownload(IntPtr lidarContext, ref RaycastResults res)
  {
      int pointCount = -1;
      CheckError(NativeMethods.Internal_GetPoints(IntPtr.Zero, lidarContext, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, ref pointCount));
      res.pointCount = pointCount;
  }

  public void SyncAndDownload(IntPtr lidarContext, ref RaycastResults res)
  {
    // May by risky, but should be faster than Marshal.Copy.
    unsafe {
      fixed (Point3f* pXYZ = res.xyz) {
      fixed (byte* p12 = res.rosPCL12, p24 = res.rosPCL24, p48 = res.rosPCL48)  {
          CheckError(NativeMethods.Internal_GetPoints(IntPtr.Zero, lidarContext, (IntPtr) pXYZ, (IntPtr) p12, (IntPtr) p24, (IntPtr) p48, ref res.pointCount));
      }
      }
    }
  }

  private void NativeHandleCheck()
  {
  }

  private void Destroy()
  {
      NativeMethods.Internal_DestroyNativeRaycaster(IntPtr.Zero);
  }

  private bool disposed;
}

} // namespace GPULidarRaycaster
