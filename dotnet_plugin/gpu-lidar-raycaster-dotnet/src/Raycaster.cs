using System;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Collections.Generic;

namespace GPULidarRaycaster
{

public class Raycaster : IDisposable
{
  private IntPtr m_NativeRaycaster = IntPtr.Zero;
  private float[] m_flatFloatBuffer;
  private IntPtr m_resultsRaw;
  private Dictionary<string, Point4f[]> m_resultBuffers;

  public Raycaster()
  {
    m_flatFloatBuffer = new float[0];
    m_resultBuffers = new Dictionary<string, Point4f[]>();
    m_resultsRaw = new IntPtr();
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

  public void Raycast(in LidarSource source, ref RaycastResults res, double timestamp)
  {
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_Raycast(
        m_NativeRaycaster,
        source.source_id,
        source.lidarPose,
        source.postRaycastTransform,
        source.sourcePoses,
        source.sourcePoses.Length,
        source.lidarArrayRingIds,
        source.lidarArrayRingIds.Length,
        source.range
    ));


    int pointCount = -1;
    CheckError(NativeMethods.Internal_GetPoints(m_NativeRaycaster, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, ref pointCount, timestamp));

    res.pointCount = pointCount;
    res.xyz = new Point3f[pointCount];
    res.rosPCL12 = new byte[12 * pointCount];
    res.rosPCL24 = new byte[24 * pointCount];
    res.rosPCL48 = new byte[48 * pointCount];

    // May by risky, but should be faster than Marshal.Copy.
    unsafe {
        fixed (Point3f* pXYZ = res.xyz) {
        fixed (byte* p12 = res.rosPCL12, p24 = res.rosPCL24, p48 = res.rosPCL48)  {
            CheckError(NativeMethods.Internal_GetPoints(m_NativeRaycaster, (IntPtr) pXYZ, (IntPtr) p12, (IntPtr) p24, (IntPtr) p48, ref pointCount, timestamp));
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
