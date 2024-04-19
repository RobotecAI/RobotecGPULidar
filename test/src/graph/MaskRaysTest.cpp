#include "helpers/testPointCloud.hpp"
#include "helpers/commonHelpers.hpp"
#include "RGLFields.hpp"
#include "helpers/sceneHelpers.hpp"
#include "helpers/lidarHelpers.hpp"

class MaskRaysTest : public RGLTest
{

protected:
	const int maskCount = 100;
	std::vector<int32_t> points_mask;
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, IS_HIT_I32, INTENSITY_F32, IS_GROUND_I32};

	void initializeMask(int raysCount)
	{
		points_mask.resize(raysCount);
		std::fill(points_mask.begin(), points_mask.end(), 1);
		for (int i = 0; i < maskCount; i++) {
			points_mask[i] = 0;
		}
	}
};

TEST_F(MaskRaysTest, invalid_argument_node) { EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_mask(nullptr, nullptr, 0)); }

TEST_F(MaskRaysTest, invalid_argument_mask)
{
	rgl_node_t raytraceNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_mask(raytraceNode, nullptr, 0));
}

TEST_F(MaskRaysTest, invalid_argument_count)
{
	rgl_node_t raytraceNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace_configure_mask(raytraceNode, points_mask.data(), 0));
}

TEST_F(MaskRaysTest, valid_arguments)
{
	rgl_node_t raytraceNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));

	initializeMask( 100);

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_mask(raytraceNode, points_mask.data(), 1));
}

TEST_F(MaskRaysTest, use_case)
{
	// Scene
	rgl_entity_t wall = makeEntity(makeCubeMesh());
	rgl_mat3x4f wallPose = Mat3x4f::TRS(Vec3f(0,0,0), Vec3f(0, 0, 0), Vec3f(10,10,10)).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(wall, &wallPose));

	// Rays
	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);

	// Mask
	initializeMask(rays.size());

	// Graph
	rgl_node_t useRaysNode = nullptr;
	rgl_node_t raytraceNode = nullptr;
	rgl_node_t yieldNode = nullptr;

	// Prepare graph without filtering
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
	TestPointCloud outputPointCloud = TestPointCloud::createFromNode(yieldNode, fields);
	auto fullCloudSize = outputPointCloud.getPointCount();

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_mask(raytraceNode, points_mask.data(), 100));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));
	TestPointCloud outputPointCloudmasked = TestPointCloud::createFromNode(yieldNode, fields);
	auto maskedCloudSize = outputPointCloud.getPointCount();

	EXPECT_EQ(fullCloudSize - maskCount, maskedCloudSize);

}