#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/lidarHelpers.hpp>

#include <math/Mat3x4f.hpp>
#include <Logger.hpp>

#if RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

class GraphFullLinearTest : public RGLTest
{};

TEST_F(GraphFullLinearTest, FullLinear)
{
	setupBoxesAlongAxes();

	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, shear = nullptr, compact = nullptr,
	           downsample = nullptr;

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({5, 5, 5}, {45, 45, 45}).toRGL();
	rgl_mat3x4f shearTf = Mat3x4f::shear({0, 0}, {-1, -1}, {0, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&shear, &shearTf));

#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&downsample, 0.1f, 0.1f, 0.1f));
#endif

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, shear));

#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(shear, downsample));
#endif

	EXPECT_RGL_SUCCESS(rgl_graph_run(useRays));
#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(downsample, "minimal.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}