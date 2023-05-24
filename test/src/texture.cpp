#include <gtest/gtest.h>
#include "utils.hpp"
#include "scenes.hpp"
#include "lidars.hpp"
#ifdef RGL_BUILD_ROS2_EXTENSION
#include "rgl/api/extensions/ros2.h"
#endif


class TextureTest : public RGLTest {

protected:

	template<typename T>
	static T *generateTexture(int width, int height)
	{
		// Generate a sample texture with a grid pattern 16x16.
		int xGridSize=width/16;
		int yGridSize=height/16;
		int xStep=0;
		int yStep=0;

		T *texels = new T[width * height];

		for (int i = 0; i < width; ++i)
		{
			for (int j = 0; j < height; ++j)
			{
				texels[i*width + j] = yStep * 0.5f + xStep * 0.5f;
				if(j % yGridSize == 0)
					yStep += yGridSize;
			}
			yStep = 0;
			if(i % xGridSize == 0)
				xStep += xGridSize;
		}
		return texels;
	}
};

TEST_F(TextureTest, rgl_texture_create)
{
	rgl_texture_t texture;
	rgl_mesh_t mesh;
	rgl_entity_t entity;

	// Create mesh with assigned texture.
	auto textureRawData = generateTexture<float>(256, 256);
	mesh = makeCubeMesh();

	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData, RGL_TEXTURE_TYPE_FLOAT, 256, 128, 1));
	EXPECT_RGL_SUCCESS(rgl_mesh_set_tex_coord(mesh, cubeUVs, 8));

	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(tape_entity_set_intensity_texture(entity, texture));

	//Create RGL graph pipeline.
	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, yieldNode = nullptr;

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 360, 0.36, 0.36);

	std::vector<rgl_field_t> yieldFields = {
			XYZ_F32,
			INTENSITY_F32,
			IS_HIT_I32
	};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	// Infinite loop to publish pointcloud to ROS2 topic for visualization purposes. Uncomment to use. Use wisely.
#ifdef RGL_BUILD_ROS2_EXTENSION

//	rgl_node_t formatNode = nullptr, ros2publishNode = nullptr;
//
//	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, yieldFields.data(), yieldFields.size()));
//	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2publishNode, "pointcloud", "rgl"));
//
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, formatNode));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2publishNode));
//
//	while(true)
//		{
//			EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
//			printf("Publishing pointcloud...\n");
//		}

#endif

}