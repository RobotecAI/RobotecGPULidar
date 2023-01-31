#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <rgl/api/core.h>
#include <scenes.hpp>

using namespace ::testing;

#define VERTICES cubeVertices
#define INDICES cubeIndices
// TODO(nebraszka): What's the best way to handle this FILENAME without copying this directive to each test?
#define FILENAME (strrchr(__FILE__, std::filesystem::path::preferred_separator) ? strrchr(__FILE__, std::filesystem::path::preferred_separator) + 1 : __FILE__)

static bool loggingConfigured = std::invoke([]() {
	std::filesystem::path logFilePath { std::filesystem::temp_directory_path() / std::filesystem::path(FILENAME).concat(".log") };
	rgl_configure_logging(RGL_LOG_LEVEL_DEBUG, logFilePath.c_str(), false);
	return true;
});

class APISurfaceTests : public RGLAutoCleanUp {};

TEST_F(APISurfaceTests, rgl_get_version_info)
{
	int major, minor, patch;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(nullptr, nullptr, nullptr), "out_major != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  nullptr, nullptr), "out_minor != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  &minor,  nullptr), "out_patch != nullptr");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_EQ(major, RGL_VERSION_MAJOR);
	EXPECT_EQ(minor, RGL_VERSION_MINOR);
	EXPECT_EQ(patch, RGL_VERSION_PATCH);
}

TEST_F(APISurfaceTests, rgl_get_last_error_string)
{
	// Invalid args
	rgl_get_last_error_string(nullptr);
	Logger::getOrCreate().flush();

	// Expected log should be written in the file
	// TODO(nebraszka): Is it possible to handle access to the log file name more nicely?
	std::filesystem::path logFilePath { std::filesystem::temp_directory_path() / std::filesystem::path(FILENAME).concat(".log") };
	ASSERT_THAT(std::filesystem::exists(logFilePath), IsTrue());
	std::string logFile = readFileStr(logFilePath);
	ASSERT_FALSE(logFile.empty());
	EXPECT_THAT(logFile, HasSubstr("rgl_get_last_error_string"));
	EXPECT_THAT(logFile, HasSubstr("warn"));

	// Valid args
	const char* error_text;
	rgl_get_last_error_string(&error_text);
	EXPECT_THAT(error_text, NotNull());
}

TEST_F(APISurfaceTests, rgl_mesh_create_destroy)
{
	rgl_mesh_t mesh = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(nullptr, nullptr,   0,                    nullptr, 0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   nullptr,   0,                    nullptr, 0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   VERTICES,  0,                    nullptr, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   VERTICES,  ARRAY_SIZE(VERTICES), nullptr, 0), "indices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   VERTICES,  ARRAY_SIZE(VERTICES), INDICES, 0), "index_count > 0");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_mesh_create(&mesh, VERTICES, ARRAY_SIZE(VERTICES), INDICES, ARRAY_SIZE(INDICES)));
	ASSERT_THAT(mesh, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_mesh_destroy(mesh));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_destroy(mesh), "Object does not exist: Mesh");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_destroy(nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_destroy((rgl_mesh_t) 0x1234), "Object does not exist: Mesh 0x1234");
}

TEST_F(APISurfaceTests, rgl_mesh_update_vertices)
{
	rgl_mesh_t mesh = makeCubeMesh();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(nullptr, nullptr,  0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    nullptr,  0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    VERTICES, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    VERTICES, ARRAY_SIZE(VERTICES) + 1), "vertex counts do not match");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_update_vertices((rgl_mesh_t) 0x1234, VERTICES, ARRAY_SIZE(VERTICES)), "Object does not exist: Mesh 0x1234");

	// Correct update_vertices
	ASSERT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES)));
}

TEST_F(APISurfaceTests, rgl_entity_create_destroy)
{
	rgl_mesh_t mesh = makeCubeMesh();
	rgl_entity_t entity = nullptr;

	// Invalid args, note: scene can be nullptr here.
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(nullptr, nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(&entity, nullptr, nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_create(&entity, (rgl_scene_t) 0x1234, mesh), "Object does not exist: Scene 0x1234");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_create(&entity, nullptr, (rgl_mesh_t) 0x1234), "Object does not exist: Mesh 0x1234");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	ASSERT_THAT(entity, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_entity_destroy(entity));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_destroy(entity), "Object does not exist: Entity");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_destroy(nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_destroy((rgl_entity_t) 0x1234), "Object does not exist: Entity 0x1234");
}

TEST_F(APISurfaceTests, rgl_entity_set_pose)
{
	rgl_entity_t entity = makeEntity();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(entity,  nullptr), "transform != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_set_pose((rgl_entity_t) 0x1234, &identity), "Object does not exist: Entity 0x1234");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identity));
}

TEST_F(APISurfaceTests, rgl_node_rays_from_mat3x4f)
{
	rgl_node_t node = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(nullptr, nullptr, 0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&node, nullptr, 0), "rays != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&node, &identity, 0), "ray_count > 0");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&node, &identity, 1));
}

// TEST_F(APISurfaceTests, rgl_node_rays_set_ring_ids)
// {
// 	rgl_node_t node = {}; 
// 	const int32_t ring_ids = 1; 
// 	const int32_t invalid_ring_ids_count = 0;
// 	const int32_t valid_ring_ids_count = 1;
	

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(nullptr, &ring_ids, valid_ring_ids_count), "nodeRawPtr != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&node, nullptr, valid_ring_ids_count), "ring_ids != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&node, &ring_ids, invalid_ring_ids_count), "ring_ids_count > 0");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&node, &ring_ids, valid_ring_ids_count));
// }

// TEST_F(APISurfaceTests, rgl_node_rays_transform)
// {
// 	rgl_node_t node = {};
// 	rgl_mat3x4f transform;

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, &transform), "nodeRawPtr != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(&node, nullptr), "transform != nullptr");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&node, &transform));
// }

// TEST_F(APISurfaceTests, rgl_node_points_transform)
// {
// 	rgl_node_t node = {};
// 	rgl_mat3x4f transform;

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(nullptr, &transform), "nodeRawPtr != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(&node, nullptr), "transform != nullptr");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&node, &transform));
// }

// TEST_F(APISurfaceTests, rgl_node_raytrace)
// {
// 	rgl_node_t node = {}; 
// 	rgl_scene_t scene; 
// 	float range = 1.0f;

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, scene, range), "nodeRawPtr != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(&node, scene, 0.0f), "range > 0.0f");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&node, nullptr, range));
// 	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&node, scene, range));
// }

// TEST_F(APISurfaceTests, rgl_node_points_format)
// {
// 	rgl_node_t node = {};
// 	rgl_field_t fields;
// 	int32_t field_count = 1;

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, &fields, field_count), "nodeRawPtr != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&node, nullptr, field_count), "fields != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&node, &fields, 0), "field_count > 0");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_points_format(&node, &fields, field_count));
// }

// TEST_F(APISurfaceTests, rgl_node_points_yield)
// {
// 	rgl_node_t node = {};
// 	rgl_field_t fields;
// 	int32_t field_count = 1;

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(nullptr, &fields, field_count), "nodeRawPtr != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&node, nullptr, field_count), "fields != nullptr");
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&node, &fields, 0), "field_count > 0");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&node, &fields, field_count));
// }

// TEST_F(APISurfaceTests, rgl_node_points_compact)
// {
// 	rgl_node_t node = {};

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "nodeRawPtr != nullptr");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&node));
// }

// TEST_F(APISurfaceTests, rgl_node_points_downsample)
// {
// 	rgl_node_t node = {};
// 	float leaf_size_x = 1.0f; 
// 	float leaf_size_y = 2.0f;
// 	float leaf_size_z = 3.0f;

// 	// Invalid args
// 	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(nullptr, leaf_size_x, leaf_size_y, leaf_size_z), "nodeRawPtr != nullptr");

// 	// This are passing, assume they should fail if size less or eq to 0??
// 	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, 0.0f, leaf_size_y, leaf_size_z), "leaf_size_x > 0.0f");
// 	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, -1.0f, leaf_size_y, leaf_size_z), "leaf_size_x > 0.0f");
// 	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, leaf_size_x, 0.0f, leaf_size_z), "leaf_size_y > 0.0f");
// 	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, leaf_size_x, leaf_size_y, 0.0f), "leaf_size_z > 0.0f");

// 	// Valid args
// 	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&node, leaf_size_x, leaf_size_y, leaf_size_z));
// }