#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>
#include <RGLFields.hpp>
#include <rgl/api/extensions/tape_api.h>

#include <math/Mat3x4f.hpp>

class Graph : public RGLAutoCleanupTest {};

TEST_F(Graph, TapeFormatNodeResults)
{
	rgl_tape_record_begin("recording");

	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, format = nullptr;

	std::vector<rgl_mat3x4f> rays = {
		Mat3x4f::TRS({ 0, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.1, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.2, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.3, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.4, 0, 0 }).toRGL()
	};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::identity().toRGL();
	std::vector<rgl_field_t> formatFields = {
		XYZ_F32,
		PADDING_32
	};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	struct FormatStruct {
		Field<XYZ_F32>::type xyz;
		Field<PADDING_32>::type padding;
	} formatStruct;

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData { outCount };
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], rays[i].value[0][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[1], rays[i].value[1][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[2], 1, 1e-6);
	}

	rgl_tape_record_end();

	rgl_cleanup();

	rgl_tape_play("recording");

	useRays = nullptr;
	raytrace = nullptr;
	lidarPose = nullptr;
	format = nullptr;

	rays = {
		Mat3x4f::TRS({ 0, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.1, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.2, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.3, 0, 0 }).toRGL(),
		Mat3x4f::TRS({ 0.4, 0, 0 }).toRGL()
	};
	lidarPoseTf = Mat3x4f::identity().toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	outCount = 0;
	outSizeOf = 0;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData_2 { static_cast<size_t>(outCount) };
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData_2.data()));

	for (int i = 0; i < formatData_2.size(); ++i) {
		EXPECT_NEAR(formatData_2[i].xyz[0], rays[i].value[0][3], 1e-6);
		EXPECT_NEAR(formatData_2[i].xyz[1], rays[i].value[1][3], 1e-6);
		EXPECT_NEAR(formatData_2[i].xyz[2], 1, 1e-6);
	}
}
