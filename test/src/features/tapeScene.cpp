#include <filesystem>

#include <helpers/sceneHelpers.hpp>
#include <helpers/commonHelpers.hpp>

#include <RGLFields.hpp>
#include <rgl/api/extensions/tape.h>
#include <math/Mat3x4f.hpp>

class Tape : public RGLTest
{};

void testCubeSceneOnGraph()
{
	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, format = nullptr;

	std::vector<rgl_mat3x4f> rays = {Mat3x4f::TRS({0, 0, 0}).toRGL(), Mat3x4f::TRS({0.1, 0, 0}).toRGL(),
	                                 Mat3x4f::TRS({0.2, 0, 0}).toRGL(), Mat3x4f::TRS({0.3, 0, 0}).toRGL(),
	                                 Mat3x4f::TRS({0.4, 0, 0}).toRGL()};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::identity().toRGL();
	std::vector<rgl_field_t> formatFields = {XYZ_F32, PADDING_32};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	struct FormatStruct
	{
		Field<XYZ_F32>::type xyz;
		Field<PADDING_32>::type padding;
	} formatStruct;

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData{static_cast<long unsigned int>(outCount)};
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], rays[i].value[0][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[1], rays[i].value[1][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[2], 1, 1e-6);
	}
}

TEST_F(Tape, SceneReconstruction)
{
	std::string cubeSceneRecordPath{
	    (std::filesystem::temp_directory_path() / std::filesystem::path("cubeSceneRecord")).string()};
	rgl_tape_record_begin(cubeSceneRecordPath.c_str());
	auto mesh = makeCubeMesh();
	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));
	rgl_tape_record_end();

	testCubeSceneOnGraph();

	rgl_cleanup();

	rgl_tape_play(cubeSceneRecordPath.c_str());

	testCubeSceneOnGraph();
}
