
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

        void assertDistance(Point4f p1, Point4f p2, float eps = 0.1f)
        {
            var distance = MathF.Sqrt(
              MathF.Pow((p1.x - p2.x), 2) +
              MathF.Pow((p1.y - p2.y), 2) +
              MathF.Pow((p1.z - p2.z), 2)
            );
            Assert.InRange<float>(distance, 0f, eps);
        }

        [Fact]
        public void InstancesTest()
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();
            var translate = new float[12] {
              1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f, 5.0f,
              0.0f, 0.0f, 1.0f, 0.0f
            };
            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "cube.obj");
            Mesh cube = ml.GetMesh(0);
            cube.id = "cube";
            lidar.AddMesh(cube);

            Mesh cubeTranslated = ml.GetMesh(0);
            cubeTranslated.id = "cube_translated";
            cubeTranslated.transform = translate;
            lidar.AddMesh(cubeTranslated);

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
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=-1.0f, y=0.0f, z=0.0f});

            ls = new LidarSource()
            {
              source_id = "test_lidar",
              source_pos = new Point3f() { x = -30.0f, y = 5.0f, z = 0.0f },
              directions = new Point3f[1] {new Point3f() { x = 1.0f, y = 0.0f, z = 0.0f }},
              range = 100.0f
            };
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=-1.0f, y=5.0f, z=0.0f});
        }

        [Fact]
        public void TransformTest()
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();

            var originTransform = new float[12] {
              1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f, 0.0f
            };

            var sinT = MathF.Sin(MathF.PI/2); //90*: 1
            var cosT = MathF.Cos(MathF.PI/2); //90*: 0

            var xRotate = new float[12] {
              1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, cosT, -sinT, 0.0f,
              0.0f, sinT, cosT, 0.0f
            };

            var yRotate = new float[12] {
              cosT, 0.0f, sinT, 0.0f,
              0.0f, 1.0f, 0.0f, 0.0f,
              -sinT, 0.0f, cosT, 0.0f
            };

            var zRotate = new float[12] {
              cosT, -sinT, 0.0f, 0.0f,
              sinT, cosT, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f, 0.0f
            };

            var xTranslate = new float[12] {
              1.0f, 0.0f, 0.0f, 5.0f,
              0.0f, 1.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f, 0.0f
            };
            var yTranslate = new float[12] {
              1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f, 5.0f,
              0.0f, 0.0f, 1.0f, 0.0f
            };
            var zTranslate = new float[12] {
              1.0f, 0.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f, 5.0f
            };

            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "cylinder_stand.obj");
            Mesh cyl = ml.GetMesh(0);
            cyl.id = "cyl";
            cyl.transform = originTransform;
            lidar.AddMesh(cyl);
            LidarSource ls = new LidarSource()
            {
              source_id = "test_lidar",
              source_pos = new Point3f() { x = -30.0f, y = 0.01f, z = 0.49f },
              directions = new Point3f[1] {new Point3f() { x = 1.0f, y = 0.0f, z = 0.0f }},
              range = 100.0f
            };

            // Origin
            Console.WriteLine("\nOrigin rotation");
            RaycastResults rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=0.0f, y=0.0f, z=0.5f});

            // Rotation around X axis
            Console.WriteLine("\nX rotation");
            lidar.UpdateMeshTransform("cyl", xRotate);
            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=-0.5f, y=0.0f, z=0.5f});
            
            // Rotation around Y axis
            Console.WriteLine("\nY rotation");
            lidar.UpdateMeshTransform("cyl", yRotate);
            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=0.0f, y=0.0f, z=0.5f});

            // Rotation around Z axis
            Console.WriteLine("\nZ rotation");
            lidar.UpdateMeshTransform("cyl", zRotate);
            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=-5.0f, y=0.0f, z=0.5f});

            // Translation on X axis
            Console.WriteLine("\nX translation");
            lidar.UpdateMeshTransform("cyl", xTranslate);
            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(1 == rr.points.Length);
            assertDistance(rr.points[0], new Point4f() {x=5.0f, y=0.0f, z=0.5f});

            // Translation on Y axis
            Console.WriteLine("\nY translation");
            lidar.UpdateMeshTransform("cyl", yTranslate);
            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(0 == rr.points.Length);

            // Translation on Z axis
            Console.WriteLine("\nZ translation");
            lidar.UpdateMeshTransform("cyl", zTranslate);
            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            Assert.True(0 == rr.points.Length);
        }

        [Fact]
        public void RemoveMesh()
        {
            Raycaster lidar = new Raycaster();
            ModelLoader ml = new ModelLoader();

            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "2cubes.obj");
            Mesh twoCubes = ml.GetMesh(0);
            twoCubes.id = "2cubes";
            lidar.AddMesh(twoCubes);

            ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "cube.obj");
            Mesh cube = ml.GetMesh(0);
            cube.id = "cube";
            lidar.AddMesh(cube);

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

            rr = new RaycastResults();
            lidar.RemoveMesh("cube");
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);
            // No rayhits expected
            Assert.True(0 == rr.points.Length);
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
              lidar.AddMesh(m);
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
            assertDistance(rr.points[0], new Point4f() {x=-1.0f, y=0.0f, z=0.0f}, 0.05f);

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
              lidar.AddMesh(m);
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

            // No hit is expected
            Assert.True(0 == rr.points.Length);

            ls = new LidarSource()
            {
              source_id = "test_lidar",
              source_pos = new Point3f() { x = -30.0f, y = 0.0f, z = 2.0f },
              directions = new Point3f[1] {new Point3f() { x = 1.0f, y = 0.0f, z = 0.0f }},
              range = 100.0f
            };

            rr = new RaycastResults();
            lidar.Raycast(ls, ref rr);
            PrintHits(rr);

            // One hit is expected
            Assert.True(1 == rr.points.Length);

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
              lidar.AddMesh(m);
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
              assertDistance(rr.points[i], new Point4f() {x=expected_point.x, y=expected_point.y, z=expected_point.z}, 0.05f);
            }

            lidar.Dispose();
        }
    }
}
