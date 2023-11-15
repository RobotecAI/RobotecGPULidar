#include <RGLFields.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>

class GaussianNoiseDistanceNodeTest : public RGLTest
{
protected:
	rgl_node_t gaussianNoiseNode;

	GaussianNoiseDistanceNodeTest() { gaussianNoiseNode = nullptr; }
};

TEST_F(GaussianNoiseDistanceNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(nullptr, 0.0f, 0.0f, 0.0f), "node != nullptr");
}

TEST_F(GaussianNoiseDistanceNodeTest, invalid_argument_st_dev_base)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.0f, -1.0f, 0.0f), "st_dev_base >= 0");
}

TEST_F(GaussianNoiseDistanceNodeTest, invalid_argument_st_dev_rise_per_meter)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.0f, 0.0f, -1.0f),
	                            "st_dev_rise_per_meter >= 0");
}

TEST_F(GaussianNoiseDistanceNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));

	// If (*gaussianNoiseNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 0.1f, 0.1f, 0.01f));
}

// The aim of the test is to verify that gaussian noise is not dependent on the ray's pose.
// Test will compare point clouds captured inside cubes.
// Cubes are spawned in different locations. Raytrace will always be performed from the same pose in relation to the cube.
// Gaussian noise is configured with zero standard deviation so there is no randomization.
TEST_F(GaussianNoiseDistanceNodeTest, same_mean_applied_in_different_poses)
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
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	ASSERT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, noiseMean, 0.0f, 0.0f));
	ASSERT_RGL_SUCCESS(rgl_node_points_transform(&toLidarFrame, &lidarTfInv));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, gaussianNoiseNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(gaussianNoiseNode, toLidarFrame));

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
