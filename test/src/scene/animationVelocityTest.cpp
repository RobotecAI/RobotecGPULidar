#include <thread>

#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/testPointCloud.hpp>

#include <Time.hpp>

/*
 * This test validates the calculated velocity of the hit point on the entity resulting from the entity's animation.
 * Animated entity is a cube, where the second half of its vertices are deformed.
 * The sensor is located inside the cube and cast one ray in known direction.
 *      Frame 0                       Frame 1
 *                                ---------------
 *  ---------------               |      ^      |
 *  |             |               |      |      |
 *  |             |      ->       |      |      |
 *  |    Sensor   |               |    Sensor   |
 *  |             |               |             |
 *  ---------------               ---------------
 */

// Transforms of the cube and sensor world adopted in tests.
// It makes sure velocities are calculated correctly regardless of the transform in the world.
const std::vector<Mat3x4f> WorldTfs = {
    Mat3x4f::identity(),
    Mat3x4f::translation(10.0f, 20.0f, 30.0f),
    Mat3x4f::rotationDeg(10.0f, 20.0f, 30.0f),
    Mat3x4f::TRS({10.0f, 20.0f, 30.0f}, {10.0f, 20.0f, 30.0f}),
};

// Animations of the cube adopted in tests
const std::vector<Mat3x4f> AnimationTfs = {
    Mat3x4f::identity(),
    Mat3x4f::translation(0.0f, 0.0f, 0.5f),
    Mat3x4f::translation(0.0f, 0.0f, -0.5f),
    Mat3x4f::translation(0.0f, 0.5f, 0.0f),
    Mat3x4f::translation(0.0f, -0.5f, 0.0f),
    // No rotations, because it is hard to predict new hit-point and calculate velocity of it.
};

class AnimationVelocityTest : public RGLTest
{
public:
	void setupGraph()
	{
		rgl_mat3x4f rayRgl = ray.toRGL();
		ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&raysNode, &rayRgl, 1));
		rgl_mat3x4f identityTfRgl = Mat3x4f::identity().toRGL();
		ASSERT_RGL_SUCCESS(rgl_node_rays_transform(&raysTransformNode, &identityTfRgl));
		ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
		ASSERT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, yieldFields.data(), yieldFields.size()));

		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysNode, raysTransformNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raysTransformNode, raytraceNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));
	}

	// Transforms sensor to world frame, triggers it and validates relative and absolute velocities.
	void runTest(const Mat3x4f& worldTf, const Vec3f& expectedRelVelocity)
	{
		rgl_mat3x4f rayWorldTfRgl = worldTf.toRGL();
		ASSERT_RGL_SUCCESS(rgl_node_rays_transform(&raysTransformNode, &rayWorldTfRgl));
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

		auto outPointCloud = TestPointCloud::createFromNode(yieldNode, yieldFields);
		ASSERT_EQ(outPointCloud.getPointCount(), 1);

		auto outRelVelocity = outPointCloud.getFieldValue<RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32>(0);
		EXPECT_NEAR(outRelVelocity.x(), expectedRelVelocity.x(), EPSILON_F);
		EXPECT_NEAR(outRelVelocity.y(), expectedRelVelocity.y(), EPSILON_F);
		EXPECT_NEAR(outRelVelocity.z(), expectedRelVelocity.z(), EPSILON_F);

		auto expectedAbsVelocity = worldTf.rotation() * expectedRelVelocity;
		auto outAbsVelocity = outPointCloud.getFieldValue<RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32>(0);
		EXPECT_NEAR(outAbsVelocity.x(), expectedAbsVelocity.x(), EPSILON_F);
		EXPECT_NEAR(outAbsVelocity.y(), expectedAbsVelocity.y(), EPSILON_F);
		EXPECT_NEAR(outAbsVelocity.z(), expectedAbsVelocity.z(), EPSILON_F);
	}

private:
	Mat3x4f ray{Mat3x4f::identity()};
	std::vector<rgl_field_t> yieldFields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32,
	                                        RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32};

	rgl_node_t raysNode{nullptr};
	rgl_node_t raysTransformNode{nullptr};
	rgl_node_t raytraceNode{nullptr};
	rgl_node_t yieldNode{nullptr};
};

TEST_F(AnimationVelocityTest, skeleton_animation_velocity)
{
	rgl_mesh_t skeletonAnimCubeMesh = makeCubeMesh();
	rgl_entity_t skeletonAnimEntity = nullptr;
	ASSERT_RGL_SUCCESS(rgl_entity_create(&skeletonAnimEntity, nullptr, skeletonAnimCubeMesh));

	// First 4 vertices are affected by bone 0, next 4 vertices are affected by bone 1. Bone 1 will be animated.
	std::vector<rgl_bone_weights_t> cubeBoneWeights = {
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {0, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {0, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {0, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {0, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {1, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {1, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {1, 0, 0, 0}},
	    {.weights = {1.0f, 0.0f, 0.0f, 0.0f}, .bone_idxes = {1, 0, 0, 0}},
	};
	std::vector<Mat3x4f> restposes = {Mat3x4f::identity(), Mat3x4f::identity()};
	std::vector<rgl_mat3x4f> restposesRgl = {restposes[0].toRGL(), restposes[1].toRGL()};

	ASSERT_RGL_SUCCESS(rgl_mesh_set_bone_weights(skeletonAnimCubeMesh, cubeBoneWeights.data(), cubeBoneWeights.size()));
	ASSERT_RGL_SUCCESS(rgl_mesh_set_restposes(skeletonAnimCubeMesh, restposesRgl.data(), restposesRgl.size()));

	setupGraph();

	auto deltaTime = Time::seconds(1);
	int frameCounter = 0;
	for (auto&& worldTf : WorldTfs) {
		for (auto&& animationTf : AnimationTfs) {
			// Frame 0: Place entity on the scene in world coordinates. Reset animation.
			ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, deltaTime.asNanoseconds() * frameCounter));
			std::vector<rgl_mat3x4f> poseFrame0Rgl = {
			    (worldTf * restposes[0]).toRGL(),
			    (worldTf * restposes[1]).toRGL(),
			};
			ASSERT_RGL_SUCCESS(rgl_entity_set_pose_world(skeletonAnimEntity, poseFrame0Rgl.data(), poseFrame0Rgl.size()));
			++frameCounter;

			// Frame 1: Animate entity in world coordinates
			ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, deltaTime.asNanoseconds() * frameCounter));
			std::vector<rgl_mat3x4f> poseFrame1Rgl = {
			    (worldTf * restposes[0]).toRGL(),
			    (worldTf * animationTf * restposes[1]).toRGL(),
			};
			ASSERT_RGL_SUCCESS(rgl_entity_set_pose_world(skeletonAnimEntity, poseFrame1Rgl.data(), poseFrame1Rgl.size()));

			// The expected relative velocity of the hit-point is the same as the translation part of the animation
			// (the sensor and cube are always in the same orientation)
			ASSERT_EQ(Mat3x4f::identity(), animationTf.rotation()); // Rotation cannot be applied for the current logic
			auto expectedRelVelocity = animationTf.translation() / static_cast<float>(deltaTime.asSeconds());
			runTest(worldTf, expectedRelVelocity);
			++frameCounter;
		}
	}
}

TEST_F(AnimationVelocityTest, external_animation_velocity)
{
	rgl_mesh_t cubeMesh = makeCubeMesh();
	rgl_entity_t externalAnimEntity = nullptr;
	ASSERT_RGL_SUCCESS(rgl_entity_create(&externalAnimEntity, nullptr, cubeMesh));

	setupGraph();

	auto deltaTime = Time::seconds(1);
	int frameCounter = 0;
	for (auto&& worldTf : WorldTfs) {
		for (auto&& animationTf : AnimationTfs) {
			// Frame 0: Place entity on the scene in world coordinates. Reset animation.
			ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, deltaTime.asNanoseconds() * frameCounter));
			ASSERT_RGL_SUCCESS(rgl_entity_apply_external_animation(externalAnimEntity, cubeVertices, ARRAY_SIZE(cubeVertices)));
			auto worldTfRgl = worldTf.toRGL();
			ASSERT_RGL_SUCCESS(rgl_entity_set_transform(externalAnimEntity, &worldTfRgl));
			++frameCounter;

			// Frame 1: Animate entity in model coordinates
			ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, deltaTime.asNanoseconds() * frameCounter));
			Vec3f animatedCubeVertices[ARRAY_SIZE(cubeVertices)];
			memcpy(&animatedCubeVertices, &cubeVertices, sizeof(cubeVertices));
			for (int i = ARRAY_SIZE(cubeVertices) / 2; i < ARRAY_SIZE(cubeVertices); ++i) { // For second half of cube vertices
				animatedCubeVertices[i] = animationTf * animatedCubeVertices[i];
			}
			ASSERT_RGL_SUCCESS(rgl_entity_apply_external_animation(
			    externalAnimEntity, reinterpret_cast<rgl_vec3f*>(animatedCubeVertices), ARRAY_SIZE(cubeVertices)));
			// Transform entity to world coordinates
			ASSERT_RGL_SUCCESS(rgl_entity_set_transform(externalAnimEntity, &worldTfRgl));

			// The expected relative velocity of the hit-point is the same as the translation part of the animation
			// (the sensor and cube are always in the same orientation)
			ASSERT_EQ(Mat3x4f::identity(), animationTf.rotation()); // Rotation cannot be applied for the current logic
			auto expectedRelVelocity = animationTf.translation() / static_cast<float>(deltaTime.asSeconds());
			runTest(worldTf, expectedRelVelocity);
			++frameCounter;
		}
	}
}
