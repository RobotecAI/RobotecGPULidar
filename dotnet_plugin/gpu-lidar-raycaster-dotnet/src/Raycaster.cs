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

  public void Raycast(in LidarSource source, ref RaycastResults res)
  {
    NativeHandleCheck();
    CheckError(NativeMethods.Internal_Raycast(m_NativeRaycaster, source.source_id,
     source.source_pos, source.directions, source.directions.Length, source.range));

    // Get points right away - this could also be done in a different time

    int results_count = 0;
    CheckError(NativeMethods.Internal_GetPoints(m_NativeRaycaster, ref m_resultsRaw, ref results_count));

    if (!m_resultBuffers.ContainsKey(source.source_id)) {
      m_resultBuffers.Add(source.source_id, new Point4f[results_count]);
    }
    else {
      var buffer_size = m_resultBuffers[source.source_id].Length;
      if (buffer_size != results_count) {
        m_resultBuffers[source.source_id] = new Point4f[results_count];
      }
    }

    res.lidar_id = source.source_id;
    res.points = m_resultBuffers[source.source_id];
    if (results_count > 0) {
      int single_point_number_of_floats = Marshal.SizeOf(res.points[0]) / sizeof(float);
      int floats_total = results_count * single_point_number_of_floats;
      if (m_flatFloatBuffer.Length < floats_total) {
        m_flatFloatBuffer = new float[floats_total];
      }
      Marshal.Copy(m_resultsRaw, m_flatFloatBuffer, 0, floats_total);
      for (int i = 0; i < results_count; ++i) {
        int flat_offset = i * single_point_number_of_floats;
        res.points[i].x = m_flatFloatBuffer[flat_offset];
        res.points[i].y = m_flatFloatBuffer[flat_offset + 1];
        res.points[i].z = m_flatFloatBuffer[flat_offset + 2];
        res.points[i].i = m_flatFloatBuffer[flat_offset + 3];
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
