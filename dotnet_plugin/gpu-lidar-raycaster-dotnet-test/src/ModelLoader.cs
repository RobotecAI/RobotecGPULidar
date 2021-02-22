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

        public GPULidarRaycaster.Model GetMesh(int mesh_id) 
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
            IntPtr triangleMesh = Internal_GetTriangleMesh(
              m_NativeModelLoader,
              mesh_id,
              ref vertices,
              ref normals,
              ref texture_coordinates,
              ref indices,
              ref vertices_size,
              ref normals_size,
              ref texture_coordinates_size,
              ref indices_size);
            GPULidarRaycaster.Model model = new GPULidarRaycaster.Model();
            model.vertices = new GPULidarRaycaster.Vector3f[vertices_size];
            model.normals = new GPULidarRaycaster.Vector3f[normals_size];
            model.texture_coordinates = new GPULidarRaycaster.Vector2f[texture_coordinates_size];
            model.indices = new GPULidarRaycaster.Vector3i[indices_size];

            float[] vertices_coords_list = new float[vertices_size*3];
            Marshal.Copy(vertices, vertices_coords_list, 0, vertices_size*3);
            for (int i = 0; i < vertices_size; ++i) {
              model.vertices[i].x = vertices_coords_list[3*i];
              model.vertices[i].y = vertices_coords_list[3*i+1];
              model.vertices[i].z = vertices_coords_list[3*i+2];
            }
            float[] normals_coords_list = new float[normals_size*3];
            Marshal.Copy(normals, normals_coords_list, 0, normals_size*3);
            for (int i = 0; i < normals_size; ++i) {
              model.normals[i].x = normals_coords_list[3*i];
              model.normals[i].y = normals_coords_list[3*i+1];
              model.normals[i].z = normals_coords_list[3*i+2];
            }
            float[] texture_coordinates_list = new float[texture_coordinates_size*2];
            Marshal.Copy(texture_coordinates, texture_coordinates_list, 0, texture_coordinates_size*2);
            for (int i = 0; i < texture_coordinates_size; ++i) {
              model.texture_coordinates[i].x = texture_coordinates_list[2*i];
              model.texture_coordinates[i].y = texture_coordinates_list[2*i+1];
            }
            int[] indices_list = new int[indices_size*3];
            Marshal.Copy(indices, indices_list, 0, indices_size*3);
            for (int i = 0; i < indices_size; ++i) {
              model.indices[i].x = indices_list[3*i];
              model.indices[i].y = indices_list[3*i+1];
              model.indices[i].z = indices_list[3*i+2];
            }
            // DEBUG
            /*
            for (int i = 0; i < vertices_size; ++i) {
              Console.WriteLine("v: (" + model.vertices[i].x + ',' + model.vertices[i].y + ',' +model.vertices[i].z + ')');
            }

            for (int i = 0; i < normals_size; ++i) {
              Console.WriteLine("n: (" + model.normals[i].x + ',' + model.normals[i].y + ',' + model.normals[i].z + ')');
            }

            for (int i = 0; i < indices_size; ++i) {
              Console.WriteLine("i: (" + model.indices[i].x + ',' + model.indices[i].y + ',' + model.indices[i].z + ')');
            }

            for (int i = 0; i < texture_coordinates_size; ++i) {
              Console.WriteLine("t: (" + model.texture_coordinates[i].x + ',' + model.texture_coordinates[i].y + ')');
            }
            */
            
            return model;
          } else {
            return new GPULidarRaycaster.Model();
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
        private static extern IntPtr Internal_GetTriangleMesh(IntPtr obj, [In] int mesh_id,
          ref IntPtr vertices, ref IntPtr normals, ref IntPtr texture_coordinates, ref IntPtr indices,
          ref int vertices_size, ref int normals_size, ref int texutres_size, ref int indices_size);


        [DllImport("libnative_model_loader.so", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Internal_LoadModel(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string path);

        
    }

}