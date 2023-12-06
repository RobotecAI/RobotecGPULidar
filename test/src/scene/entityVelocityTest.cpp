#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <api/apiCommon.hpp>

#include <thread>
#include <ranges>
#include <chrono>

using namespace std::chrono_literals;

struct TestScene
{
	struct DynamicCube
	{
		DynamicCube(std::function<void(DynamicCube&, float)> updateFn) : updateFn(updateFn)
		{
			EXPECT_RGL_SUCCESS(
			    rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
			EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
		}

		void update(float t) { updateFn(*this, t); }

		rgl_mesh_t mesh{};
		rgl_entity_t entity{};
		std::function<void(DynamicCube&, float)> updateFn;
	};

	static TestScene build()
	{
		// Setup scene
		constexpr float DIST = 5.0f;

		// +X, moves +/- 1 from initial position
		DynamicCube translatingCube([](DynamicCube& cube, float t) {
			auto pose = Mat3x4f::translation(Vec3f{DIST + std::sin(t), 0, 0}).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));
		});

		// -X, rotates around Z, ccw
		DynamicCube rotatingCube([](DynamicCube& cube, float t) {
			auto pose = (Mat3x4f::translation(-DIST, 0, 0) * Mat3x4f::rotationRad(0, 0, t)).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));
		});

		// +Y, scales between 0.5 and 1.5
		DynamicCube scalingCube([](DynamicCube& cube, float t) {
			float s = 1.0f + std::sin(t) / 2.0f;
			auto pose = (Mat3x4f::translation(0, DIST, 0) * Mat3x4f::scale(s, s, s)).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));
		});

		// -Y, oscillates between cube and pyramid by collapsing frontal face into cube's center point
		DynamicCube morphingCube([](DynamicCube& cube, float t) {
			auto tt = t / 2.0f;
			auto pose = Mat3x4f::translation(Vec3f{0, -DIST, 0}).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));

			rgl_vec3f changed[ARRAY_SIZE(cubeVertices)];
			for (int i = 0; i < ARRAY_SIZE(cubeVertices); ++i) {
				changed[i] = cubeVertices[i];
			}

			changed[2].value[0] += -abs(sin(tt));
			changed[2].value[1] += -abs(sin(tt));
			changed[2].value[2] -= -abs(sin(tt));
			changed[3].value[0] -= -abs(sin(tt));
			changed[3].value[1] += -abs(sin(tt));
			changed[3].value[2] -= -abs(sin(tt));
			changed[6].value[0] += -abs(sin(tt));
			changed[6].value[1] += -abs(sin(tt));
			changed[6].value[2] += -abs(sin(tt));
			changed[7].value[0] -= -abs(sin(tt));
			changed[7].value[1] += -abs(sin(tt));
			changed[7].value[2] += -abs(sin(tt));
			EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(cube.mesh, changed, ARRAY_SIZE(changed)));
		});

		// -Z; moved around a circle on XY plane, rotating around Z, so that it always points to the center
		DynamicCube rotateTranslateCube([](DynamicCube& cube, float t) {
			constexpr float R = 1.0f;
			constexpr float s = 0.33f;
			auto translation = Mat3x4f::translation(R * std::sin(t), -R * std::cos(t), -CUBE_HALF_EDGE);
			auto rotation = Mat3x4f::rotationRad(0, 0, t);
			auto scale = Mat3x4f::scale(s, s, s);
			auto pose = (translation * rotation * scale).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));
		});

		// +Z, translated +-Z and morphed (-Z wall oscillates along Z), effects should cancel out
		DynamicCube morphTranslateCube([](DynamicCube& cube, float t) {
			constexpr float S = 2.0f; // scale
			auto pose = (Mat3x4f::translation({-DIST, -DIST, DIST + -sin(t)}) * Mat3x4f::scale(S, S, S)).toRGL();
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));

			// Note: vertices are changed in local space, so they are not affected by the entity's pose
			rgl_vec3f changed[ARRAY_SIZE(cubeVertices)];
			for (int i = 0; i < ARRAY_SIZE(cubeVertices); ++i) {
				changed[i] = cubeVertices[i];
			}
			changed[0].value[2] += sin(t) / S;
			changed[1].value[2] += sin(t) / S;
			changed[2].value[2] += sin(t) / S;
			changed[3].value[2] += sin(t) / S;
			EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(cube.mesh, changed, ARRAY_SIZE(changed)));
		});

		TestScene scene = {
		    .cubes = {translatingCube, rotatingCube, scalingCube, morphingCube, rotateTranslateCube, morphTranslateCube}
        };

		return scene;
	}

	void update(float currentTimeSeconds)
	{
		for (auto& cube : cubes) {
			cube.update(currentTimeSeconds);
		}
	}

	std::vector<DynamicCube> cubes;
};

/**
 * This test is meant to be run together with RViz2 to visualize results.
 * - linear velocity
 * - angular velocity
 * - mesh updates
 * - ray source velocity
 * *) outputs = {RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, RGL_FIELD_RADIAL_SPEED_F32}
 */
#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#include <graph/NodesRos2.hpp>
TEST(EntityVelocity, Interactive)
{
	//	GTEST_SKIP(); // Comment to run the test
	rgl_node_t rays = nullptr, rayTransform = nullptr, raytrace = nullptr, compact = nullptr, format = nullptr,
	           publish = nullptr, markers = nullptr, transformPoints = nullptr;
	std::vector<rgl_mat3x4f> raysTf = makeLidar3dRays(360.0f, 180.0f);
	raysTf.resize(1);
	rgl_mat3x4f lidarPoseRGL = Mat3x4f::identity().toRGL(), lidarIndicatorPoseRGL;

	// Published fields
	std::vector<rgl_field_t> fields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32,
	                                   RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, RGL_FIELD_RADIAL_SPEED_F32};

	// Construct graph: rays -> transform ->  raytrace -> compact -> format -> publish
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&rayTransform, &lidarPoseRGL));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPoints, &lidarPoseRGL));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&publish, "EntityVelocityTest", "world"));
	createOrUpdateNode<Ros2PublishPointVelocityMarkersNode>(&markers, "EntityVelocityTestMarkers", "world");
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, rayTransform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rayTransform, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPoints));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPoints, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPoints, markers));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, publish));

	// Test Scene
	TestScene scene = TestScene::build();
	rgl_entity_t lidarIndicator = makeEntity();

	// Main loop
	uint64_t frameId = 0;
	uint64_t nsPerFrame = 10 * 1000 * 1000; // 10 ms per frame
	scene.update(0);
	while (true) {
		uint64_t currentTime = frameId * nsPerFrame;
		float currentTimeSeconds = static_cast<double>(currentTime) / 1E9;
		float deltaTime = static_cast<float>(nsPerFrame) / 1E9f;
		Vec3f sensorLinearVelocityXYZ = Vec3f{0, 0, sin(currentTimeSeconds)}; // units
		Vec3f sensorAngularVelocityXYZ = Vec3f{0, 0, 0};                      // degrees

		Mat3x4f diff = Mat3x4f::TRS(sensorLinearVelocityXYZ * deltaTime, sensorAngularVelocityXYZ * deltaTime);
		Mat3x4f lidarPose = diff * Mat3x4f::fromRGL(lidarPoseRGL);
		Mat3x4f lidarIndicatorPose = Mat3x4f::translation(lidarPose.translation() + Vec3f{0, 0, -0.2f}) * lidarPose.rotation() *
		                             Mat3x4f::scale(0.01f, 0.01f, 0.01f);
		lidarPoseRGL = lidarPose.toRGL();
		lidarIndicatorPoseRGL = lidarIndicatorPose.toRGL();

		ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(lidarIndicator, &lidarIndicatorPoseRGL));

		auto pointsTransform = Mat3x4f::identity().toRGL();
		EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&rayTransform, &lidarPoseRGL));
		EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPoints, &pointsTransform));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace_in_motion(&raytrace, nullptr,
		                                               reinterpret_cast<const rgl_vec3f*>(&sensorLinearVelocityXYZ),
		                                               reinterpret_cast<const rgl_vec3f*>(&sensorAngularVelocityXYZ), false));

		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

		std::this_thread::sleep_for(10ms);
		frameId += 1;

		int32_t count = 0, size = 0;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(compact, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &count, &size));
		std::vector<Vec3f> xyz(count), absVelocities(count), relVelocities(count), radialSpeeds(count);
		if (count == 0) {
			continue;
		}
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compact, RGL_FIELD_XYZ_VEC3_F32, xyz.data()));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compact, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, absVelocities.data()));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compact, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, relVelocities.data()));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(compact, RGL_FIELD_RADIAL_SPEED_F32, radialSpeeds.data()));
		//		fmt::print("xyz: {}\n", repr(xyz.data(), xyz.size()));
		//		fmt::print("absVelocity: {}\n", repr(absVelocities.data(), absVelocities.size()));
		//		fmt::print("relVelocities: {}\n", repr(relVelocities.data(), relVelocities.size()));
	}
}
#endif
