
using System;
using GPULidarRaycaster;

namespace GPULidarTest
{
    public class GPULidarSimpleTest
    {
        public static void Main(string[] args)
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();

            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "test.obj");
            var meshes_num = ml.GetNumberOfMeshes();

            Console.WriteLine("Adding meshes..");
            for (int i = 0; i < meshes_num; ++i) {
              Mesh m = ml.GetMesh(i);
              m.id = i.ToString();
              Console.WriteLine("--- Mesh # " + m.id);
              Console.WriteLine("Vertices " + m.vertices.Length);
              Console.WriteLine("Normals " + m.normals.Length);
              Console.WriteLine("Texture coords " + m.texture_coordinates.Length);
              Console.WriteLine("Indices " + m.indices.Length);
              lidar.AddOrUpdateMesh(m);
            }
            Console.WriteLine("---");
            Console.WriteLine("All meshes added.");

            LidarSource ls = new LidarSource()
            {
              source_id = "test_lidar",
              source_pos = new Point3f() { x = 0, y = 0, z = 0 },
              directions = new Point3f[3] { new Point3f() { x = 1, y = 0, z = 0 },
                                            new Point3f() { x = 0, y = 1, z = 0 },
                                            new Point3f() { x = 0, y = 0, z = 1 }
                                          },
              range = 10
            };

            RaycastResults rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            if (rr.points.Length > 0) {
              Console.WriteLine("Results len: " + rr.points.Length + " fist point " + rr.points[0].x + ", " + rr.points[0].y +
                                ", " + rr.points[0].z + ", " + rr.points[0].i);
            }
            lidar.Dispose();
        }
    }
}
