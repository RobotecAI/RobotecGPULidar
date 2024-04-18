#include "helpers/commonHelpers.hpp"
#include "helpers/sceneHelpers.hpp"
#include "helpers/mathHelpers.hpp"
#include "helpers/lidarHelpers.hpp"

#include "RGLFields.hpp"


using namespace ::testing;

class EntityTest : public RGLTest
{};

TEST_F(EntityTest, rgl_entity_create_destroy)
{
	rgl_mesh_t mesh = makeCubeMesh();
	rgl_entity_t entity = nullptr;

	// Invalid args, note: scene can be nullptr here.
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(nullptr, nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(&entity, nullptr, nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(&entity, (rgl_scene_t) 0x1234, mesh), "scene == nullptr");
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

TEST_F(EntityTest, rgl_entity_set_pose)
{
	rgl_entity_t entity = makeEntity();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(entity, nullptr), "transform != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_set_pose((rgl_entity_t) 0x1234, &identityTestTransform), "Entity 0x1234");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identityTestTransform));
}