using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace GPULidarTest
{
    public class ModelLoader : IDisposable
    {
        private IntPtr m_NativeModelLoader = IntPtr.Zero;
        public ModelLoader()
        {
            m_NativeModelLoader = Internal_CreateNativeModelLoader();
        }

        public void LoadModel(string path)
        {
          if (m_NativeModelLoader != IntPtr.Zero) {
            Console.WriteLine("Loading model from " + path);
            IntPtr mesh = Internal_LoadModel(m_NativeModelLoader, path);
          }
        }

        public int GetNumberOfMeshes()
        {
          if (m_NativeModelLoader != IntPtr.Zero) {
            return Internal_GetNumberOfMeshes(m_NativeModelLoader);
          } else {
            return 0;
          }
        }

        public GPULidarRaycaster.Mesh GetMesh(int mesh_index)
        {
          if (m_NativeModelLoader != IntPtr.Zero) {
            IntPtr vertices = new IntPtr();
            IntPtr normals = new IntPtr();
            IntPtr texture_coordinates = new IntPtr();;
            IntPtr indices = new IntPtr();;
            int vertices_size = new int();
            int normals_size = new int();
            int texture_coordinates_size = new int();
            int indices_size = new int();
            Internal_GetTriangleMesh(
              m_NativeModelLoader,
              mesh_index,
              ref vertices,
              ref normals,
              ref texture_coordinates,
              ref indices,
              ref vertices_size,
              ref normals_size,
              ref texture_coordinates_size,
              ref indices_size);
            GPULidarRaycaster.Mesh mesh = new GPULidarRaycaster.Mesh();
            mesh.vertices = new GPULidarRaycaster.Vector3f[vertices_size];
            mesh.normals = new GPULidarRaycaster.Vector3f[normals_size];
            mesh.texture_coordinates = new GPULidarRaycaster.Vector2f[texture_coordinates_size];
            mesh.indices = new GPULidarRaycaster.Vector3i[indices_size];

            float[] vertices_coords_list = new float[vertices_size*3];
            Marshal.Copy(vertices, vertices_coords_list, 0, vertices_size*3);
            for (int i = 0; i < vertices_size; ++i) {
              mesh.vertices[i].x = vertices_coords_list[3*i];
              mesh.vertices[i].y = vertices_coords_list[3*i+1];
              mesh.vertices[i].z = vertices_coords_list[3*i+2];
            }
            float[] normals_coords_list = new float[normals_size*3];
            Marshal.Copy(normals, normals_coords_list, 0, normals_size*3);
            for (int i = 0; i < normals_size; ++i) {
              mesh.normals[i].x = normals_coords_list[3*i];
              mesh.normals[i].y = normals_coords_list[3*i+1];
              mesh.normals[i].z = normals_coords_list[3*i+2];
            }
            float[] texture_coordinates_list = new float[texture_coordinates_size*2];
            Marshal.Copy(texture_coordinates, texture_coordinates_list, 0, texture_coordinates_size*2);
            for (int i = 0; i < texture_coordinates_size; ++i) {
              mesh.texture_coordinates[i].x = texture_coordinates_list[2*i];
              mesh.texture_coordinates[i].y = texture_coordinates_list[2*i+1];
            }
            int[] indices_list = new int[indices_size*3];
            Marshal.Copy(indices, indices_list, 0, indices_size*3);
            for (int i = 0; i < indices_size; ++i) {
              mesh.indices[i].x = indices_list[3*i];
              mesh.indices[i].y = indices_list[3*i+1];
              mesh.indices[i].z = indices_list[3*i+2];
            }
            // DEBUG
            /*
            for (int i = 0; i < vertices_size; ++i) {
              Console.WriteLine("v: (" + mesh.vertices[i].x + ',' + mesh.vertices[i].y + ',' +mesh.vertices[i].z + ')');
            }

            for (int i = 0; i < normals_size; ++i) {
              Console.WriteLine("n: (" + mesh.normals[i].x + ',' + mesh.normals[i].y + ',' + mesh.normals[i].z + ')');
            }

            for (int i = 0; i < indices_size; ++i) {
              Console.WriteLine("i: (" + mesh.indices[i].x + ',' + mesh.indices[i].y + ',' + mesh.indices[i].z + ')');
            }

            for (int i = 0; i < texture_coordinates_size; ++i) {
              Console.WriteLine("t: (" + mesh.texture_coordinates[i].x + ',' + mesh.texture_coordinates[i].y + ')');
            }
            */

            return mesh;
          } else {
            return new GPULidarRaycaster.Mesh();
          }
        }

        public void Dispose()
        {
            Dispose(true);
        }

        private void Dispose(bool disposing)
        {
            if (!disposed)
            {
                if (disposing)
                {
                    // Dispose managed resources.
                }

                Destroy();
                disposed = true;
            }
        }

        private void Destroy()
        {
            if (m_NativeModelLoader != IntPtr.Zero)
            {
                Internal_DestroyNativeModelLoader(m_NativeModelLoader);
                m_NativeModelLoader = IntPtr.Zero;
            }
        }

        private bool disposed;

        [DllImport("libnative_model_loader.so", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Internal_CreateNativeModelLoader();

        [DllImport("libnative_model_loader.so", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Internal_DestroyNativeModelLoader(IntPtr obj);

        [DllImport("libnative_model_loader.so", CallingConvention = CallingConvention.Cdecl)]
        private static extern int Internal_GetNumberOfMeshes(IntPtr obj);

        [DllImport("libnative_model_loader.so", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Internal_GetTriangleMesh(IntPtr obj, [In] int mesh_index,
          ref IntPtr vertices, ref IntPtr normals, ref IntPtr texture_coordinates, ref IntPtr indices,
          ref int vertices_size, ref int normals_size, ref int texutres_size, ref int indices_size);

        [DllImport("libnative_model_loader.so", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Internal_LoadModel(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string path);
    }
}
