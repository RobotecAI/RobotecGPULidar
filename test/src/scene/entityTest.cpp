#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/mathHelpers.hpp>
#include <helpers/lidarHelpers.hpp>

#include <RGLFields.hpp>

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

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

TEST_F(EntityTest, rgl_entity_set_id)
{
	rgl_entity_t entity = makeEntity();
	int validID = 1;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_id(nullptr, validID), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_id(entity, RGL_ENTITY_INVALID_ID), "id != RGL_ENTITY_INVALID_ID");

	// Correct set_id
	EXPECT_RGL_SUCCESS(rgl_entity_set_id(entity, validID));
}

// Test spawns 3 boxes along Y axis:
// box1 and box2 with specific entity ID assigned, box3 with no entity ID (RGL should assign default ID in that case).
// After raytracing, the point cloud is validated to have a proper value of the field ENTITY_ID.
// Optionally, the test publishes point cloud to the ROS2 topic for visualization purposes (the code needs to be uncommented).
//
//      box1      box2      box3
//
//      ----      ----      ----
//      |  |      |  |      |  |
//      ----      ----      ----
//
//
//                 xx
//                LIDAR
TEST_F(EntityTest, entity_id_use_case)
{
	constexpr int BOX1_ID = 1;
	constexpr int BOX2_ID = 2;
	// box3 with no ID assignment

	constexpr float BOX1_Y_POS = -5;
	constexpr float BOX2_Y_POS = 0;
	constexpr float BOX3_Y_POS = 5;

	constexpr float BOX1_BOX2_BORDER = (BOX1_Y_POS + BOX2_Y_POS) / 2.0f;
	constexpr float BOX2_BOX3_BORDER = (BOX2_Y_POS + BOX3_Y_POS) / 2.0f;

	rgl_node_t useRaysNode = nullptr, raytraceNode = nullptr, yieldNode = nullptr;
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.72, 0.36);

	std::vector<rgl_field_t> yieldFields = {XYZ_VEC3_F32, ENTITY_ID_I32, IS_HIT_I32};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	std::vector<::Field<XYZ_VEC3_F32>::type> outPoints;
	std::vector<::Field<ENTITY_ID_I32>::type> outID;
	std::vector<::Field<IS_HIT_I32>::type> outIsHit;

	outPoints.resize(rays.size());
	outID.resize(rays.size());
	outIsHit.resize(rays.size());

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, XYZ_VEC3_F32, outPoints.data()));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, ENTITY_ID_I32, outID.data()));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldNode, IS_HIT_I32, outIsHit.data()));

	for (int i = 0; i < rays.size(); ++i) {
		// Non-hit should have id equals INVALID_ENTITY_ID
		if (!outIsHit[i]) {
			EXPECT_EQ(outID[i], RGL_ENTITY_INVALID_ID);
			continue;
		}

		// box1
		if (outPoints[i].y() < BOX1_BOX2_BORDER) {
			EXPECT_EQ(outID[i], BOX1_ID);
			continue;
		}

		// box2
		if (outPoints[i].y() > BOX1_BOX2_BORDER && outPoints[i].y() < BOX2_BOX3_BORDER) {
			EXPECT_EQ(outID[i], BOX2_ID);
			continue;
		}

		// box3 - it hasn't had ID assigned manually. RGL should set DEFAULT_ENTITY_ID
		if (outPoints[i].y() > BOX2_BOX3_BORDER) {
			EXPECT_EQ(outID[i], RGL_DEFAULT_ENTITY_ID);
			continue;
		}
	}

// Infinite loop to publish point cloud to ROS2 topic for visualization purposes. Uncomment to use. Use wisely.
#ifdef RGL_BUILD_ROS2_EXTENSION
//	rgl_node_t formatNode = nullptr;
//	rgl_node_t ros2publishNode = nullptr;
//
//	EXPECT_RGL_SUCCESS(rgl_node_points_format(&formatNode, yieldFields.data(), yieldFields.size()));
//	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2publishNode, "pointcloud", "world"));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(yieldNode, formatNode));
//	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(formatNode, ros2publishNode));
//	while (true) {
//		// In each frame randomize one of the ids to ensure changes are visible & arbitrary ids are handled correctly.
//		EXPECT_RGL_SUCCESS(rgl_entity_set_id(box2, 1 + rand() % 100));
//		EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
//	}
#endif
}