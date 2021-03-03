
using System;
using GPULidarRaycaster;
using Xunit;

namespace GPULidarTest
{
    public class GPURaycastingTest
    {
        void PrintHits(RaycastResults rr)
        {
            Console.WriteLine("Resulted " + rr.points.Length + " hits:");
            foreach(var point in rr.points) {
              Console.WriteLine("Hitpoint: (" + point.x + ',' + point.y + ',' + point.z + ')');
            }
        }

        [Fact]
        public void SimpleRaycast()
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();

            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "cube.obj");
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
              source_pos = new Point3f() { x = -30.0f, y = 0.0f, z = 0.0f },
              directions = new Point3f[1] {new Point3f() { x = 1.0f, y = 0.0f, z = 0.0f }},
              range = 100.0f
            };

            RaycastResults rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);

            PrintHits(rr);

            // Only one rayhit expected
            Assert.True(1 == rr.points.Length);

            // Hitpoint is expected to be near (-1,0,0)
            var distance = MathF.Sqrt(
              MathF.Pow((rr.points[0].x + 1), 2) +
              MathF.Pow((rr.points[0].y), 2) +
              MathF.Pow((rr.points[0].z), 2)
            );
            Assert.InRange<float>(distance, 0f, 0.01f);

            lidar.Dispose();
        }

        [Fact]
        public void SimpleEmptyRaycast()
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();

            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "2cubes.obj");
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
              source_pos = new Point3f() { x = 2, y = 0, z = 0 },
              directions = new Point3f[1] {new Point3f() { x = -1, y = 0, z = 0 }},
              range = 10
            };

            RaycastResults rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);

            // No hit is expected
            Assert.True(0 == rr.points.Length);

            lidar.Dispose();
        }

        [Fact]
        public void InvertedSphereMulticast()
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();

            // Inverted sphere is a 1m radius sphere centered in (0,0,0) with normals flipped
            // inside.
            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "inverted_sphere.obj");
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

            // Calculate random points on the sphere, these will be directions
            // and hitpoints at the same time.
            var random_directions = new Point3f[100];
            var random = new System.Random();
            for (int i = 0; i < 100; ++i) {
                var x_ = random.NextDouble();
                var y_ = random.NextDouble();
                var z_ = random.NextDouble();
                var norm_distance_ = 1/(MathF.Sqrt(
                  MathF.Pow((float)x_, 2) +
                  MathF.Pow((float)y_, 2) +
                  MathF.Pow((float)z_, 2)
                ));
                random_directions[i].x = (float)x_ * norm_distance_;
                random_directions[i].y = (float)y_ * norm_distance_;
                random_directions[i].z = (float)z_ * norm_distance_;
            }

            LidarSource ls = new LidarSource()
            {
              source_id = "test_lidar",
              source_pos = new Point3f() { x = 0, y = 0, z = 0 },
              directions = random_directions,
              range = 10
            };

            RaycastResults rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);

            PrintHits(rr);

            // 100 hits are expected
            Assert.True(100 == rr.points.Length);

            // Each hit should match a direction point
            for (int i = 0; i < 100; ++i) {
              var expected_point = random_directions[i];
              var distance = MathF.Sqrt(
                MathF.Pow((rr.points[i].x - expected_point.x), 2) +
                MathF.Pow((rr.points[i].y - expected_point.y), 2) +
                MathF.Pow((rr.points[i].z - expected_point.z), 2)
              );
              Assert.InRange<float>(distance, 0f, 0.15f);
            }

            lidar.Dispose();
        }
    }
}
