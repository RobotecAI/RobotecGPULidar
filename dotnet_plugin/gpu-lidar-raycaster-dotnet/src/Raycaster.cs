using System;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Collections.Generic;

namespace GPULidarRaycaster
{

public class Raycaster : IDisposable
{
  private IntPtr m_NativeRaycaster = IntPtr.Zero;

  public Raycaster()
  {
    CheckError(NativeMethods.Internal_CreateNativeRaycaster(out m_NativeRaycaster));
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

  public void AddMesh(in Mesh mesh)
  {
    NativeHandleCheck();
    bool hasMesh = false;
    CheckError(NativeMethods.Internal_HasMesh(m_NativeRaycaster, mesh.id, ref hasMesh));
    if (hasMesh) {
      return;
    }
    // *** *** *** ACHTUNG *** *** ***
    // This call is VERY expensive, even if the mesh is not being added (when native side detects it's been already uploaded).
    // The reason for that is that mesh.vertices performs a full copy of mesh vertices.
    // Some of used meshes have 1e5 vertices, so it's not an option to copy them on every frame.
    CheckError(NativeMethods.Internal_AddMesh(m_NativeRaycaster, mesh.id, mesh.transform, mesh.is_global, mesh.vertices,
      mesh.normals, mesh.texture_coordinates, mesh.indices, mesh.indices.Length, mesh.vertices.Length, mesh.transform.Length));
  }

  public void RemoveMesh(in string mesh_id)
  {
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_RemoveMesh(m_NativeRaycaster, mesh_id));
  }

  public void UpdateMeshTransform(in string mesh_id, in float[] transform)
  {
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_UpdateMeshTransform(m_NativeRaycaster, mesh_id, transform, transform.Length));
  }

  public IntPtr CreateLidarContext(in LidarSource source)
  {
    NativeHandleCheck();
    IntPtr ctx = IntPtr.Zero;
    CheckError(NativeMethods.Internal_CreateLidarContext(
            m_NativeRaycaster,
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
            m_NativeRaycaster,
            ctx
    ));
  }

  public void RaycastAsync(in LidarSource source, IntPtr lidarContext, double timestamp)
  {
    // At the moment timestamp is not yet propagated to the native side.
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_Raycast(
        m_NativeRaycaster,
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
      CheckError(NativeMethods.Internal_GetPoints(m_NativeRaycaster, lidarContext, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, ref pointCount));
      res.pointCount = pointCount;
  }

  public void SyncAndDownload(IntPtr lidarContext, ref RaycastResults res)
  {
    // May by risky, but should be faster than Marshal.Copy.
    unsafe {
      fixed (Point3f* pXYZ = res.xyz) {
      fixed (byte* p12 = res.rosPCL12, p24 = res.rosPCL24, p48 = res.rosPCL48)  {
          CheckError(NativeMethods.Internal_GetPoints(m_NativeRaycaster, lidarContext, (IntPtr) pXYZ, (IntPtr) p12, (IntPtr) p24, (IntPtr) p48, ref res.pointCount));
      }
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
