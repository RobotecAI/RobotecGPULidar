#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <api/apiCommon.hpp>

#include <thread>
#include <ranges>
#include <chrono>
#include <numbers>

using namespace std::chrono_literals;

constexpr float RadToDeg = (180.0f / std::numbers::pi_v<float>);

struct TestScene
{
	struct DynamicCube
	{
		DynamicCube(std::function<void(DynamicCube&, float)> updateFn) : updateFn(updateFn)
		{
			//Nebraszka: dlaczego za każdym razem tworzymy nowy mesh, zamiast użyć jednego i tworzyć z niego kolejne entities?
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
	           publish = nullptr, markersRelVel = nullptr, markersAbsVel = nullptr, transformPoints = nullptr;
	std::vector<rgl_mat3x4f> raysTf = makeLidar3dRays(360.0f, 180.0f);
	rgl_mat3x4f lidarPoseRGL = Mat3x4f::identity().toRGL(), lidarIndicatorPoseRGL; //Nebraszka: nie rozumiem tego

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
	createOrUpdateNode<Ros2PublishPointVelocityMarkersNode>(&markersRelVel, "EntityRelVelocityTestMarkers", "world",
	                                                        RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32);
	createOrUpdateNode<Ros2PublishPointVelocityMarkersNode>(&markersAbsVel, "EntityAbsVelocityTestMarkers", "world",
	                                                        RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32);
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, rayTransform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rayTransform, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPoints));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPoints, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPoints, markersRelVel));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPoints, markersAbsVel));
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
		Vec3f sensorLinearVelocityXYZ = Vec3f{0, cos(currentTimeSeconds), sin(currentTimeSeconds)}; // units
		Vec3f sensorAngularVelocityXYZ = Vec3f{0, 0, 0};                                            // radians

		Mat3x4f diff = Mat3x4f::TRS(sensorLinearVelocityXYZ * deltaTime, sensorAngularVelocityXYZ * RadToDeg * deltaTime);
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
	}
}

#endif

class EntityVelocityUnitTests : public RGLTest
{};

TEST_F(EntityVelocityUnitTests, StaticLidarStaticEntity)
{
	rgl_node_t rays = nullptr, raytrace = nullptr, compact = nullptr, format = nullptr;
	std::vector<rgl_mat3x4f> raysTf = {Mat3x4f::identity().toRGL()};
	std::vector<rgl_field_t> velocityFields = {RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32,
	                                           RGL_FIELD_RADIAL_SPEED_F32};
	auto cubePosition = Vec3f{0, 0, 5};
	spawnCubeOnScene(Mat3x4f::translation(cubePosition));

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&format, velocityFields.data(), velocityFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rays, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format));

	uint64_t currentTime = 0;
	ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));
	ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

	currentTime += 1;
	ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));
	ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSize;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(Vec3f));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(Vec3f));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_RADIAL_SPEED_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(float));

	Vec3f absVelocity, relVelocity;
	float radVelocity;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &absVelocity));
	EXPECT_EQ(absVelocity.x(), 0.0f);
	EXPECT_EQ(absVelocity.y(), 0.0f);
	EXPECT_EQ(absVelocity.z(), 0.0f);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, &relVelocity));
	EXPECT_EQ(relVelocity.x(), 0.0f);
	EXPECT_EQ(relVelocity.y(), 0.0f);
	EXPECT_EQ(relVelocity.z(), 0.0f);
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_RADIAL_SPEED_F32, &radVelocity));
	EXPECT_EQ(radVelocity, 0.0f);
}

TEST_F(EntityVelocityUnitTests, StationaryLidarTranslatingEntity)
{
	rgl_node_t rays = nullptr, raytrace = nullptr, compact = nullptr, format = nullptr;
	std::vector<rgl_mat3x4f> raysTf = {Mat3x4f::identity().toRGL()};
	std::vector<rgl_field_t> velocityFields = {RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32,
	                                           RGL_FIELD_RADIAL_SPEED_F32};
	auto cubePosition = Vec3f{0, 0, 5};
	auto cubeVelocity = Vec3f{0, 0, 2};
	rgl_entity_t cubeEntity = spawnCubeOnScene(Mat3x4f::translation(cubePosition));

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&format, velocityFields.data(), velocityFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rays, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format));

	uint64_t deltaTime = 10 * 1000 * 1000;
	uint64_t currentTime = 0;
	ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));
	ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

	for (int i = 0; i < 2; i++) {
		currentTime += deltaTime;
		ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));
		Vec3f cubeTranslVec = cubeVelocity * Vec3f{deltaTime, deltaTime, deltaTime};
		rgl_mat3x4f newPose = Mat3x4f::translation(cubePosition + cubeTranslVec).toRGL();
		ASSERT_RGL_SUCCESS(rgl_entity_set_pose(cubeEntity, &newPose));
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));
		std::this_thread::sleep_for(10ms);
	}

	int32_t outCount, outSize;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(Vec3f));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(Vec3f));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_RADIAL_SPEED_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(float));

	Vec3f absVelocity, relVelocity;
	float radVelocity;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &absVelocity));
	EXPECT_EQ(absVelocity.x(), cubeVelocity.x());
	EXPECT_EQ(absVelocity.y(), cubeVelocity.y());
	EXPECT_EQ(absVelocity.z(), cubeVelocity.z());
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, &relVelocity));
	EXPECT_EQ(relVelocity.x(), cubeVelocity.x());
	EXPECT_EQ(relVelocity.y(), cubeVelocity.y());
	EXPECT_EQ(relVelocity.z(), cubeVelocity.z());
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_RADIAL_SPEED_F32, &radVelocity));
	//    EXPECT_EQ(radVelocity, cubeVelocity);
}

constexpr float DIST = 5.0f;

TEST_F(EntityVelocityUnitTests, StationaryLidarTranslatingEntity_Working_Version)
{
	rgl_node_t rays = nullptr, raytrace = nullptr, compact = nullptr, format = nullptr;
	std::vector<rgl_mat3x4f> raysTf = {Mat3x4f::identity().toRGL()};
	std::vector<rgl_field_t> velocityFields = {RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32,
	                                           RGL_FIELD_RADIAL_SPEED_F32};

	TestScene::DynamicCube translatingCube([](TestScene::DynamicCube& cube, float t) {
		auto pose = Mat3x4f::translation(Vec3f{0, 0, DIST * t}).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &pose));
	});


	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&format, velocityFields.data(), velocityFields.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rays, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format));

	uint64_t frameId = 0;
	uint64_t nsPerFrame = 10 * 1000 * 1000; // 10 ms per frame
	for (int i = 0; i < 3; ++i) {
		auto currentTime = frameId * nsPerFrame;
		auto currentTimeSeconds = static_cast<double>(currentTime) / 1E9;
		ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));

		translatingCube.update(currentTimeSeconds);

		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));
		frameId += 1;
	}

	int32_t outCount, outSize;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(Vec3f));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(Vec3f));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_RADIAL_SPEED_F32, &outCount, &outSize));
	EXPECT_EQ(outCount, raysTf.size());
	EXPECT_EQ(outSize, sizeof(float));

	Vec3f absVelocity, relVelocity;
	float radVelocity;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, &absVelocity));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, &relVelocity));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_RADIAL_SPEED_F32, &radVelocity));
	//    EXPECT_EQ(radVelocity, cubeVelocity);
}


/*
 * Unit tests:
 *
 * RELATIVE:
 * - stationary lidar, stationary entity: relative velocity should be zero; rel_vel = abs_vel
 * - stationary lidar, moving entity: relative velocity should be equal to entity's velocity
 * - moving lidar, stationary object: relative velocity should be equal to lidar's velocity, but in opposite direction
 * - moving lidar, moving entity, same direction: relative velocity should be difference between lidar's and entity's velocity
 * - moving lidar, moving entity, opposite direction: relative velocity should be sum of lidar's and entity's velocity
 * - rotating lidar

 * RADIAL:
 * - stationary lidar, stationary entity: radial speed should be zero
 * - stationary lidar, entity moving towards lidar: radial speed should be positive
 * - stationary lidar, entity moving away from lidar: radial speed should be negative
 *
 * - rotating lidar, rotating entity (Earth, Moon)
 * - stationary lidar, entity moving perpendicular to lidar's line of sight: radial speed should be zero (izolated test)
 * - stationary lidar, entity moving at an angle to lidar's line of sight: radial speed should be not zero (cosine) - e.g. 45 deg
 */