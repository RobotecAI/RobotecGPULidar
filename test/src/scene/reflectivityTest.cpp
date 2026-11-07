#include <helpers/geometryData.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/textureHelpers.hpp>

#include <RGLFields.hpp>
struct ReflectivityTest : public RGLTestWithParam<std::tuple<float, unsigned char, float>>
{};

INSTANTIATE_TEST_SUITE_P(Parametrized, ReflectivityTest,
                         testing::Combine(testing::Values(0.012f, 0.12f, 1.23f),
                                          testing::Values(static_cast<unsigned char>(0), static_cast<unsigned char>(127),
                                                          static_cast<unsigned char>(255)),
                                          testing::Values(0.012, 0.123, 1.23)));

TEST_P(ReflectivityTest, read_value)
{
	auto [alpha, value, distance] = GetParam();

	int textureWidth = 100;
	int textureHeight = 100;

	rgl_texture_t texture = nullptr;
	rgl_entity_t entity = nullptr;
	rgl_mesh_t mesh = makeCubeMesh();
	auto textureRawData = generateStaticColorTexture<TextureTexelFormat>(textureWidth, textureHeight, value);

	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture, textureRawData.data(), textureWidth, textureHeight));
	EXPECT_RGL_SUCCESS(rgl_mesh_set_texture_coords(mesh, cubeUVs, ARRAY_SIZE(cubeUVs)));

	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_intensity_texture(entity, texture));

	// Scale the cube in order to test different distances
	rgl_mat3x4f cubeTransform = Mat3x4f::TRS({0, 0, 0}, {0, 0, 0}, {distance, distance, distance}).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(entity, &cubeTransform));

	// Create RGL graph pipeline.
	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, compactNode = nullptr, yieldNode = nullptr;

	std::vector<rgl_mat3x4f> rays = {// Ray must be incident perpendicular to the surface to receive all intensity
	                                 Mat3x4f::TRS({0, 0, 0}, {0, 0, 0}).toRGL()};

	std::vector<rgl_field_t> yieldFields = {INTENSITY_F32, REFLECTIVITY_F32, DISTANCE_F32};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactNode, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_reflectivity_alpha(raytraceNode, alpha));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	std::vector<::Field<INTENSITY_F32>::type> outIntensity;
	std::vector<::Field<REFLECTIVITY_F32>::type> outReflectivity;
	std::vector<::Field<DISTANCE_F32>::type> outDistance;

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(yieldNode, INTENSITY_F32, &outCount, &outSizeOf));
	EXPECT_EQ(outSizeOf, getFieldSize(INTENSITY_F32));

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(yieldNode, REFLECTIVITY_F32, &outCount, &outSizeOf));
	EXPECT_EQ(outSizeOf, getFieldSize(REFLECTIVITY_F32));

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(yieldNode, DISTANCE_F32, &outCount, &outSizeOf));
	EXPECT_EQ(outSizeOf, getFieldSize(DISTANCE_F32));

	outIntensity.resize(outCount);
	outReflectivity.resize(outCount);
	outDistance.resize(outCount);

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, INTENSITY_F32, outIntensity.data()));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, REFLECTIVITY_F32, outReflectivity.data()));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, DISTANCE_F32, outDistance.data()));

	for (int i = 0; i < outCount; ++i) {
		float outDistanceValue = outDistance.at(i);
		float intensity = outIntensity.at(i);
		float expectedIntensity = static_cast<float>(value) / (outDistanceValue * outDistanceValue);
		float expectedReflectivity = alpha * static_cast<float>(value);

		EXPECT_NEAR(expectedIntensity, intensity, EPSILON_F * expectedIntensity);
		EXPECT_NEAR(expectedReflectivity, outReflectivity.at(i), 1e-3f);
	}
}
