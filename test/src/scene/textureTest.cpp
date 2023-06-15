#include <gtest/gtest.h>
#include "utils.hpp"
#include "scenes.hpp"
#include "lidars.hpp"

#ifdef RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

constexpr unsigned short UsMaxValue = 255;

struct TextureTest : public RGLTestWithParam<std::tuple<int, int, unsigned short>>, public RGLPointTestHelper{};

INSTANTIATE_TEST_SUITE_P(
		Parametrized, TextureTest,
		testing::Combine(
				testing::Values(10),
				testing::Values(10),
				testing::Values(0, 1, UsMaxValue/2, UsMaxValue)
		));

TEST_F(TextureTest, rgl_texture_invalid_argument)
{
	rgl_texture_t texture;
	std::vector<unsigned char> textureRawData;

	auto initializeArgumentsLambda = [&texture, &textureRawData]() {
		texture= nullptr;
		textureRawData = generateStaticColorTexture<unsigned char>(100, 100, 100);
	};
	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_texture_create(nullptr, textureRawData.data(), 100, 100), "texture != nullptr");
	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_texture_create(&texture, nullptr, 100, 100), "texels != nullptr");
	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_texture_create(&texture, textureRawData.data(), -1, 100), "width > 0");
	initializeArgumentsLambda();
	EXPECT_RGL_INVALID_ARGUMENT(rgl_texture_create(&texture, textureRawData.data(), 100, 0), "height > 0");

}

TEST_P(TextureTest, rgl_texture_reading)
{
	auto [width, height, value] = GetParam();
	std::printf("%d",value);

	rgl_texture_t texture= nullptr;
	rgl_entity_t entity = nullptr;
	rgl_mesh_t mesh = makeCubeMesh();
	auto textureRawData = generateCheckerboardTexture<unsigned char>(width, height);

	printf("Host:\n");
	for(int i = 0; i < 10; i++)
	{
		for(int j = 0; j < 10; j++)
		{
			auto pixel = textureRawData[i * 10 + j];
			printf("%d ", pixel);
		}
		printf("\n");
	}
	printf("\n");

	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData.data(), width, height));
	EXPECT_RGL_SUCCESS(rgl_mesh_set_texture_coords(mesh, cubeUVs, ARRAY_SIZE(cubeUVs)));

	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_intensity_texture(entity, texture));

	// Create RGL graph pipeline.
	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, yieldNode = nullptr;

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 360, 0.36, 0.36);

	std::vector<rgl_field_t> yieldFields = {
			INTENSITY_F32
	};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	std::vector<::Field<INTENSITY_F32>::type> outIntensity;

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(yieldNode, INTENSITY_F32, &outCount, &outSizeOf));
	EXPECT_EQ(outSizeOf, getFieldSize(INTENSITY_F32));

	outIntensity.resize(outCount);

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, INTENSITY_F32, outIntensity.data()));

	for (int i = 0; i < outCount; ++i)
	{
		//EXPECT_NEAR(((float)value) / UsMaxValue, outIntensity.at(i), EPSILON_F);
	}

}

// Use-case test. Create texture, mesh and entity. Set texture to entity and run graph pipeline.
// As a result, we should get the cube with assigned gradient-checkerboard texture.
TEST_P(TextureTest, rgl_texture_use_case)
{
	auto [width, height, value] = GetParam();
	rgl_texture_t texture;
	rgl_mesh_t mesh;
	rgl_entity_t entity;

	// Create mesh with assigned texture.
	auto textureRawData = generateCheckerboardTexture<char>(width, height);
	mesh = makeCubeMesh();

	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData.data(), 256, 128));
	EXPECT_RGL_SUCCESS(rgl_mesh_set_texture_coords(mesh, cubeUVs, 8));

	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_intensity_texture(entity, texture));

	//Create RGL graph pipeline.
	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr;

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 360, 0.36, 0.36);

	std::vector<rgl_field_t> yieldFields = {
			XYZ_F32,
			INTENSITY_F32,
			IS_HIT_I32
	};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 1000));

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

