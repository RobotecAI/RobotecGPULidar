#include <RGLFields.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/lidarHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/testPointCloud.hpp>
class LidarVisibilityTest : public RGLTest
{
public:
	std::vector<rgl_field_t> fields = {XYZ_VEC3_F32, IS_HIT_I32, DISTANCE_F32,};
};

TEST_F(LidarVisibilityTest, UseCase)
{
	// Scene
	rgl_entity_t smallCube = makeEntity(makeCubeMesh());
	rgl_mat3x4f smallCubePose = Mat3x4f::TRS(Vec3f(0, 0, 0), Vec3f(0, 0, 0), Vec3f(1, 1, 1)).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(smallCube, &smallCubePose));

	rgl_entity_t midCube = makeEntity(makeCubeMesh());
	rgl_mat3x4f midCubePose = Mat3x4f::TRS(Vec3f(0, 0, 0), Vec3f(0, 0, 0), Vec3f(2, 2, 2)).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(midCube, &midCubePose));

	rgl_entity_t bigCube = makeEntity(makeCubeMesh());
	rgl_mat3x4f bigCubePose = Mat3x4f::TRS(Vec3f(0, 0, 0), Vec3f(0, 0, 0), Vec3f(3, 3, 3)).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_transform(bigCube, &bigCubePose));

	// Rays
	std::vector<rgl_mat3x4f> rays = makeGridOfParallelRays(Vec2f(0, 0), Vec2f(0, 0), Vec2i(1, 1));

	rgl_node_t useRaysNode = nullptr;
	rgl_node_t raytraceNode = nullptr;
	rgl_node_t compactNode = nullptr;
	rgl_node_t yieldNode = nullptr;

	// Prepare graph
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compactNode, IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldNode, fields.data(), fields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, yieldNode));

	EXPECT_RGL_SUCCESS(rgl_node_raytrace_configure_id(raytraceNode, 1));
	EXPECT_RGL_SUCCESS(rgl_entity_set_sensor_id(smallCube, 1));
	// Mid cube with unset sensor_Id
	EXPECT_RGL_SUCCESS(rgl_entity_set_sensor_id(bigCube, 2));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytraceNode));

	TestPointCloud outputPointCloud = TestPointCloud::createFromNode(yieldNode, fields);
	auto cloudSize = outputPointCloud.getPointCount();
	EXPECT_EQ(cloudSize, 1);

	auto distances = outputPointCloud.getFieldValues<DISTANCE_F32>();
	EXPECT_NEAR(distances[0], 2.0, 0.0001);
}