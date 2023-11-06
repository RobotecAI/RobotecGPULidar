#include "helpers/commonHelpers.hpp"
#include "helpers/lidarHelpers.hpp"
#include "helpers/sceneHelpers.hpp"

#include <thread>
#include <ranges>
#include <chrono>

using namespace std::chrono_literals;

/**
 * This test is meant to be run together with RViz2 to visualize results.
 * - linear velocity
 * - angular velocity
 * - mesh updates
 * - ray source velocity
 * *) outputs = {RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32, RGL_FIELD_RADIAL_SPEED_F32}
 */
#ifdef RGL_BUILD_ROS2_EXTENSION
#include "rgl/api/extensions/ros2.h"
#include "helpers/sceneHelpers.hpp"
TEST(EntityVelocity, Interactive)
{
	GTEST_SKIP(); // Comment to run the test
	rgl_node_t rays = nullptr, raytrace = nullptr, compact = nullptr, format = nullptr, publish = nullptr;
	std::vector<rgl_mat3x4f> raysTf = makeLidar3dRays(360.0f, 180.0f);

	// Published fields
	std::vector<rgl_field_t> fields = {
	    RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32,
	    //	    RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32,
	    //	    RGL_FIELD_RADIAL_SPEED_F32
	};

	// Construct graph: rays -> raytrace -> compact -> format -> publish
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rays, raysTf.data(), raysTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&publish, "EntityVelocityTest", "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(rays, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, format));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(format, publish));

	// Helper struct
	struct DynamicCube
	{
		DynamicCube(Mat3x4f startPose, std::function<void(DynamicCube&, double)> updateFn) : startPose(startPose)
		{
			auto startPoseRGL = startPose.toRGL();
			EXPECT_RGL_SUCCESS(
			    rgl_mesh_create(&mesh, cubeVertices, ARRAY_SIZE(cubeVertices), cubeIndices, ARRAY_SIZE(cubeIndices)));
			EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
			EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &startPoseRGL));
			update = [=, this](double t) { updateFn(*this, t); };
		}

		Mat3x4f startPose;
		rgl_mesh_t mesh{};
		rgl_entity_t entity{};
		std::function<void(double)> update;
	};

	// Setup scene
	constexpr float CUBE_DIST = 5.0f;

	// +X, moves +/- 1 from initial position
	DynamicCube translatingCube(Mat3x4f::translation({CUBE_DIST, 0, 0}), [](DynamicCube& cube, double t) {
		auto change = Mat3x4f::translation(Vec3f{std::sin(t), 0, 0});
		auto newPose = (cube.startPose * change).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &newPose));
	});

	// -X, rotates around Z, ccw
	DynamicCube rotatingCube(Mat3x4f::translation(-CUBE_DIST, 0, 0), [](DynamicCube& cube, double t) {
		auto change = Mat3x4f::rotationRad(0, 0, t);
		auto newPose = (cube.startPose * change).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &newPose));
	});

	// +Y, scales between 0.5 and 1.5
	DynamicCube scalingCube(Mat3x4f::translation(0, CUBE_DIST, 0), [](DynamicCube& cube, double t) {
		auto ds = 1.0f + sin(t) / 2.0f;
		auto change = Mat3x4f::scale(ds, ds, ds);
		auto newPose = (cube.startPose * change).toRGL();
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube.entity, &newPose));
	});

	// -Y, oscillates between cube and pyramid by collapsing frontal face into cube's center point
	DynamicCube morphingCube(Mat3x4f::translation(0, -CUBE_DIST, 0), [](DynamicCube& cube, double t) {
		rgl_vec3f changed[ARRAY_SIZE(cubeVertices)];
		for (int i = 0; i < ARRAY_SIZE(cubeVertices); ++i) {
			changed[i] = cubeVertices[i];
		}

		changed[2].value[0] += sin(t);
		changed[2].value[1] += sin(t);
		changed[2].value[2] -= sin(t);
		changed[3].value[0] -= sin(t);
		changed[3].value[1] += sin(t);
		changed[3].value[2] -= sin(t);
		changed[6].value[0] += sin(t);
		changed[6].value[1] += sin(t);
		changed[6].value[2] += sin(t);
		changed[7].value[0] -= sin(t);
		changed[7].value[1] += sin(t);
		changed[7].value[2] += sin(t);
		EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(cube.mesh, changed, ARRAY_SIZE(changed)));
	});

	// Main loop
	uint64_t frameId = 0;
	uint64_t nsPerFrame = 10 * 1000 * 1000; // 10 ms per frame
	while (true) {
		auto currentTime = frameId * nsPerFrame;
		auto currentTimeSeconds = static_cast<double>(currentTime) / 1E9;
		EXPECT_RGL_SUCCESS(rgl_scene_set_time(nullptr, currentTime));

		translatingCube.update(currentTimeSeconds);
		rotatingCube.update(currentTimeSeconds);
		scalingCube.update(currentTimeSeconds);
		morphingCube.update(currentTimeSeconds);

		EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
		std::this_thread::sleep_for(10ms);
		frameId += 1;
	}
}
#endif
