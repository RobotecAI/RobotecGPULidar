#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <rgl/api/core.h>
#include <scenes.hpp>

#define VERTICES cubeVertices
#define INDICES cubeIndices

using namespace ::testing;

class APISurfaceTests : public RGLTest
{};

TEST_F(APISurfaceTests, rgl_get_version_info)
{
	int major, minor, patch;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(nullptr, nullptr, nullptr), "out_major != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major, nullptr, nullptr), "out_minor != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major, &minor, nullptr), "out_patch != nullptr");

	// Valid args
	ASSERT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_EQ(major, RGL_VERSION_MAJOR);
	EXPECT_EQ(minor, RGL_VERSION_MINOR);
	EXPECT_EQ(patch, RGL_VERSION_PATCH);
}

TEST_F(APISurfaceTests, rgl_mesh_create_destroy)
{
	rgl_mesh_t mesh = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(nullptr, nullptr, 0, nullptr, 0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh, nullptr, 0, nullptr, 0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh, VERTICES, 0, nullptr, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh, VERTICES, ARRAY_SIZE(VERTICES), nullptr, 0), "indices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh, VERTICES, ARRAY_SIZE(VERTICES), INDICES, 0), "index_count > 0");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_mesh_create(&mesh, VERTICES, ARRAY_SIZE(VERTICES), INDICES, ARRAY_SIZE(INDICES)));
	ASSERT_THAT(mesh, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_mesh_destroy(mesh));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_destroy(mesh), "Mesh");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_destroy(nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_destroy((rgl_mesh_t) 0x1234), "Mesh 0x1234");
}

TEST_F(APISurfaceTests, rgl_mesh_update_vertices)
{
	rgl_mesh_t mesh = makeCubeMesh();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(nullptr, nullptr, 0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh, nullptr, 0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh, VERTICES, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES) + 1),
	                            "vertex counts do not match");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_update_vertices((rgl_mesh_t) 0x1234, VERTICES, ARRAY_SIZE(VERTICES)),
	                          "Mesh 0x1234");

	// Correct update_vertices
	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES)));
}

TEST_F(APISurfaceTests, rgl_entity_create_destroy)
{
	rgl_mesh_t mesh = makeCubeMesh();
	rgl_entity_t entity = nullptr;

	// Invalid args, note: scene can be nullptr here.
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(nullptr, nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(&entity, nullptr, nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_create(&entity, (rgl_scene_t) 0x1234, mesh), "Scene 0x1234");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_create(&entity, nullptr, (rgl_mesh_t) 0x1234), "Mesh 0x1234");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	ASSERT_THAT(entity, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_entity_destroy(entity));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_destroy(entity), "Entity");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_destroy(nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_destroy((rgl_entity_t) 0x1234), "Entity 0x1234");
}

TEST_F(APISurfaceTests, rgl_entity_set_pose)
{
	rgl_entity_t entity = makeEntity();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(entity, nullptr), "transform != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_set_pose((rgl_entity_t) 0x1234, &identityTestTransform), "Entity 0x1234");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identityTestTransform));
}

TEST_F(APISurfaceTests, rgl_entity_set_id)
{
	rgl_entity_t entity = makeEntity();
	int validID = 1;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_id(nullptr, 1), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_id(entity, RGL_ENTITY_INVALID_ID), "id != RGL_ENTITY_INVALID_ID");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_entity_set_id(entity, validID));
}
