
using System;
using GPULidarRaycaster;

namespace ConsoleApplication
{
    public class GPULidarSimpleTest
    {
        public static void Main(string[] args)
        {
            Raycaster lidar = new Raycaster();
            Model m = new Model()
            {
              id = "test",
              vertices = new Vector3f[3] { new Vector3f() { x = 1, y = 2, z = 0},
                                           new Vector3f() { x = 10, y = 12, z = 3},
                                           new Vector3f() { x = 1, y = 2, z = 3}
                                         },
              normals = new Vector3f[3] { new Vector3f() { x = 1, y = 0, z = 0},
                                          new Vector3f() { x = 0, y = 1, z = 0},
                                          new Vector3f() { x = 0, y = 0, z = 1}
                                        },
              texture_coordinates = new Vector2f[2],
              indices = new Vector3i[3] { new Vector3i() { x = 0, y = 0, z = 0},
                                          new Vector3i() { x = 1, y = 0, z = 0},
                                          new Vector3i() { x = 2, y = 0, z = 0}
                                        }
            };

            //Console.WriteLine("Vertices ", m.vertices.Length);
            lidar.AddModel(m);
            lidar.Dispose();
        }
    }
}
