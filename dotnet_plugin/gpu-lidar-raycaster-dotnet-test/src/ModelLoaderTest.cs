
using System;
using GPULidarRaycaster;
using Xunit;

namespace GPULidarTest
{
    public class ModelLoaderTest
    {
        [Fact]
        public void LoadingOBJTest()
        {
          ModelLoader ml = new ModelLoader();

          ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "cube.obj");
          var meshes_num = ml.GetNumberOfMeshes();
          Assert.Equal(1, meshes_num);

          ml.LoadModel(AppDomain.CurrentDomain.BaseDirectory + "2cubes.obj");
          meshes_num = ml.GetNumberOfMeshes();
          Assert.Equal(3, meshes_num);
        }
    }
}