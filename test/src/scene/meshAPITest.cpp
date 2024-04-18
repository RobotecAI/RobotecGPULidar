#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/mathHelpers.hpp>

#define VERTICES cubeVertices
#define INDICES cubeIndices

using namespace ::testing;

class MeshTest : public RGLTest
{};

TEST_F(MeshTest, rgl_mesh_create_destroy)
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

TEST_F(MeshTest, rgl_mesh_update_vertices)
{
	rgl_mesh_t mesh = makeCubeMesh();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(nullptr, nullptr, 0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh, nullptr, 0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh, VERTICES, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES) + 1),
	                            "vertex counts do not match");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_update_vertices((rgl_mesh_t) 0x1234, VERTICES, ARRAY_SIZE(VERTICES)), "Mesh 0x1234");

	// Correct update_vertices
	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES)));
}