#include <RGLFields.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>

class GaussianNoiseAngularRayNodeTest : public RGLTest
{
protected:
	rgl_node_t gaussianNoiseNode;

	GaussianNoiseAngularRayNodeTest() { gaussianNoiseNode = nullptr; }
};

// TODO(nebraszka): Parameterize the test to take a permutation of the set of all axes.
TEST_F(GaussianNoiseAngularRayNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(nullptr, 0.0f, 0.0f, RGL_AXIS_X), "node != nullptr");
}

TEST_F(GaussianNoiseAngularRayNodeTest, invalid_argument_st_dev)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, -1.0f, RGL_AXIS_X),
	                            "st_dev >= 0");
}

TEST_F(GaussianNoiseAngularRayNodeTest, invalid_argument_rotation_axis)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, 0.0f, (rgl_axis_t) 4),
	                            "rotation_axis");
}

TEST_F(GaussianNoiseAngularRayNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 0.0f, 0.0f, RGL_AXIS_X));

	// If (*gaussianNoiseNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, 2.0f, 2.0f, RGL_AXIS_X));
}

// The aim of the test is to verify that gaussian noise is not dependent on the ray's pose.
// Test will compare point clouds captured inside cubes.
// Cubes are spawned in different locations. Raytrace will always be performed from the same pose in relation to the cube.
// Gaussian noise is configured with zero standard deviation so there is no randomization.
TEST_F(GaussianNoiseAngularRayNodeTest, same_mean_applied_in_different_poses)
{
	float noiseMean = 0.01f;

	std::vector<Mat3x4f> cubePoses = {
	    Mat3x4f::identity(),
	    Mat3x4f::TRS({100, 200, 300}, {10, 20, 30}),
	    Mat3x4f::TRS({123, 321, 10.5f}, {90, 0, 35.6f}),
	};
	ASSERT_TRUE(cubePoses.size() > 1);

	rgl_node_t useRays = nullptr, lidarPose = nullptr, raytrace = nullptr, toLidarFrame = nullptr;

	std::vector<rgl_mat3x4f> lidarRays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarTf = cubePoses[0].toRGL();
	rgl_mat3x4f lidarTfInv = cubePoses[0].inverse().toRGL();

	rgl_entity_t cube = makeEntity(makeCubeMesh());
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(cube, &lidarTf));

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, lidarRays.data(), lidarRays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarTf));
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&gaussianNoiseNode, noiseMean, 0.0f, RGL_AXIS_Y));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_points_transform(&toLidarFrame, &lidarTfInv));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, gaussianNoiseNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(gaussianNoiseNode, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, toLidarFrame));

	bool isFirstRaytrace = true;

	std::vector<Field<XYZ_VEC3_F32>::type> firstRaytraceResult;
	std::vector<Field<XYZ_VEC3_F32>::type> lastRaytraceResult;
	firstRaytraceResult.resize(lidarRays.size());
	lastRaytraceResult.resize(lidarRays.size());

	for (auto&& cubeTf : cubePoses) {
		lidarTf = cubeTf.toRGL();
		lidarTfInv = cubeTf.inverse().toRGL();
		ASSERT_RGL_SUCCESS(rgl_entity_set_pose(cube, &lidarTf));
		ASSERT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarTf));
		ASSERT_RGL_SUCCESS(rgl_node_points_transform(&toLidarFrame, &lidarTfInv));
		ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace));

		if (isFirstRaytrace) {
			ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(toLidarFrame, XYZ_VEC3_F32, firstRaytraceResult.data()));
			isFirstRaytrace = false;
			continue;
		}

		ASSERT_RGL_SUCCESS(rgl_graph_get_result_data(toLidarFrame, XYZ_VEC3_F32, lastRaytraceResult.data()));
		for (int i = 0; i < lidarRays.size(); ++i) {
			ASSERT_NEAR(firstRaytraceResult[i].x(), lastRaytraceResult[i].x(), 1e-4);
			ASSERT_NEAR(firstRaytraceResult[i].y(), lastRaytraceResult[i].y(), 1e-4);
			ASSERT_NEAR(firstRaytraceResult[i].z(), lastRaytraceResult[i].z(), 1e-4);
		}
	}
}
