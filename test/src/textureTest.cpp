#include <gtest/gtest.h>
#include "utils.hpp"
#include "scenes.hpp"
#include "lidars.hpp"

#ifdef RGL_BUILD_ROS2_EXTENSION
#include "rgl/api/extensions/ros2.h"
#endif


#pragma region Params

template<typename T>
using ParamT = std::vector<std::tuple<int, int, T>>;

static std::tuple<ParamT<int>, ParamT<float>> validParams{

		// All the test cases in format: {width, height value}

		{ // Test cases for int
				std::make_tuple(1, 1, 0),
				std::make_tuple(2, 3, 0),
				std::make_tuple(100, 2000, 0),
				std::make_tuple(1, 1, 1),
				std::make_tuple(2, 3, 1),
				std::make_tuple(100, 2000, 1),
				std::make_tuple(1, 1, 50),
				std::make_tuple(2, 3, 50),
				std::make_tuple(100, 2000, 50),
		},
		{ // Test cases for float
				std::make_tuple(1, 1, 0.0f),
				std::make_tuple(2, 3, 0.0f),
				std::make_tuple(100, 2000, 0.0f),
				std::make_tuple(1, 1, 1.5f),
				std::make_tuple(2, 3, 1.5f),
				std::make_tuple(100, 2000, 0.5f),
				std::make_tuple(1, 1, 21.37f),
				std::make_tuple(2, 3, 21.37f),
				std::make_tuple(100, 2000, 21.37f),
		},
};

static std::tuple<ParamT<int>, ParamT<float>> invalidParams{

		// All the test cases in format: {width, height value}

		{ // Test cases for int
				std::make_tuple(-1, 1, 0),
				std::make_tuple(-2, 3, 0),
				std::make_tuple(-100, 2000, 0),
				std::make_tuple(0, 1, 1),
				std::make_tuple(0, 3, 1),
				std::make_tuple(0, 2000, 1),
		},
		{ // Test cases for float
				std::make_tuple(-1, 1, 0.0f),
				std::make_tuple(-2, 3, 0.0f),
				std::make_tuple(-100, 2000, 0.0f),
				std::make_tuple(0, 1, 1.0f),
				std::make_tuple(0, 3, 1.0f),
				std::make_tuple(0, 2000, 1.0f),
		},
};
#pragma endregion

//class TextureTest : public RGLTestWithParam<std::tuple<int, int, float>>, public RGLPointTestHelper {
template<typename T>
struct TextureTest : public RGLTest, public RGLPointTestHelper {

	TextureTest() : validArguments{std::get<ParamT<T>>(validParams)},
	                invalidArguments{std::get<ParamT<T>>(invalidParams)} {}

	ParamT<T> validArguments;
	ParamT<T> invalidArguments;

protected:




};

TYPED_TEST_SUITE_P(TextureTest);

TYPED_TEST_P(TextureTest, rgl_texture_reading) {
	for (auto const &[width, height, value]: this->validArguments) {
		rgl_texture_t texture;

		// Create texture.
		auto textureRawData = generateStaticColorTexture(width, height, value);

		EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData, RGL_TEXTURE_TYPE_FLOAT, width, height));

		// Destroy texture.
		delete[] textureRawData;
	}
}

TYPED_TEST_P(TextureTest, rgl_texture_invalid_argument) {
	for (auto const &[width, height, value]: this->invalidArguments) {
		rgl_texture_t texture;

		// Create texture.
		auto textureRawData = generateStaticColorTexture(width, height, value);


		EXPECT_RGL_INVALID_ARGUMENT(rgl_texture_create(&texture, textureRawData, RGL_TEXTURE_TYPE_FLOAT, width, height),
		                            "Invalid argument");

		// Destroy texture.
		delete[] textureRawData;
	}
}


TYPED_TEST_P(TextureTest, rgl_texture_use_case) {
	for (auto const &[width, height, value]: this->validArguments) {
		rgl_texture_t texture;
		rgl_mesh_t mesh;
		rgl_entity_t entity;

		// Create mesh with assigned texture.
		auto textureRawData = generateCheckerboardTexture<float>(width, height);
		mesh = makeCubeMesh();

		EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData, RGL_TEXTURE_TYPE_FLOAT, 256, 128));
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
		// Destroy texture.
		delete[] textureRawData;
	}
}


REGISTER_TYPED_TEST_SUITE_P(TextureTest, rgl_texture_reading, rgl_texture_invalid_argument, rgl_texture_use_case);

using Types = testing::Types<int, float>;
INSTANTIATE_TYPED_TEST_SUITE_P(TypedTests, TextureTest, Types);

