#include <helpers/commonHelpers.hpp>
#include "math/Mat3x4f.hpp"
#include "helpers/lidarHelpers.hpp"
#include "RGLFields.hpp"
#include "helpers/testPointCloud.hpp"
#include "helpers/sceneHelpers.hpp"

class GraphMultiReturn : public RGLTest
{};

TEST_F(GraphMultiReturn, basic)
{
	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, compact = nullptr, yield = nullptr;
	std::vector<rgl_field_t> mrFields = {IS_HIT_I32, DISTANCE_F32, XYZ_VEC3_F32, INTENSITY_F32, ENTITY_ID_I32};

	std::vector<rgl_mat3x4f> rays = {Mat3x4f::identity().toRGL()};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({1, 2, 3}, {0, 30, 0}).toRGL();

	rgl_entity_t cube = makeEntity();
	rgl_mat3x4f cubeTf = Mat3x4f::TRS({0, 0, 0}, {0, 0, 0}, {10, 10, 10}).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube, &cubeTf));

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yield, mrFields.data(), mrFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, yield));

	EXPECT_RGL_SUCCESS(rgl_graph_run(useRays));

	TestPointCloud pc = TestPointCloud::createFromNode(yield, mrFields);
	EXPECT_EQ(pc.getPointCount(), 1);

	EXPECT_RGL_SUCCESS(rgl_graph_destroy(useRays));
}