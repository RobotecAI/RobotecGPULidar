#include <utils.hpp>
#include <RaysTestHelper.hpp>
#include <scenes.hpp>

class DistanceFieldTest : public RGLTest, public RGLRaysNodeTestHelper {
protected:
    rgl_node_t raytraceNode, gaussianNoiseNode, yieldPointsNode;
    rgl_scene_t scene;

    DistanceFieldTest()
    {
        gaussianNoiseNode = nullptr;
        raytraceNode = nullptr;
        yieldPointsNode = nullptr;
        scene = nullptr;
    }

    void prepareScene()
    {
        rgl_mesh_t cube_mesh = makeCubeMesh();
        rgl_entity_t cube_entity = nullptr;
	    EXPECT_RGL_SUCCESS(rgl_entity_create(&cube_entity, scene, cube_mesh));
        rgl_mat3x4f entity_tf = {
		.value = {
			{ 1, 0, 0, 0 },
			{ 0, 1, 0, 0 },
			{ 0, 0, 1, 5 } }
	    }; //TODO use Mat3x4
        EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube_entity, &entity_tf));
    }
};

TEST_F(DistanceFieldTest, should_compute_distance_from_ray_beginning)
{
    SUCCEED();
}

TEST_F(DistanceFieldTest, should_change_distance_when_gaussian_angular_noise_considered)
{
    //createTestUseRaysNode(10);
    rgl_mat3x4f ray_tf = {
    .value = {
        { 1, 0, 0, 0 },
        { 0, 1, 0, 0 },
        { 0, 0, 1, 0 }}
    };

    rgl_node_rays_from_mat3x4f(&useRaysNode, &ray_tf, 1);
    EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, scene, 10));
    EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&gaussianNoiseNode, 1.0f, 0.5f, 0.5f));
    rgl_field_t fields[] = {DISTANCE_F32};
    EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldPointsNode, fields, 1));

    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, gaussianNoiseNode));
    EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(gaussianNoiseNode, yieldPointsNode));
   
    prepareScene();

	EXPECT_RGL_SUCCESS(rgl_graph_run(yieldPointsNode));

    int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(yieldPointsNode, DISTANCE_F32, &outCount, &outSizeOf));
    std::vector<::Field<DISTANCE_F32>::type> outData;
    outData.resize(outCount);

	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yieldPointsNode, DISTANCE_F32, outData.data()));

	std::cerr << "Got your distance!" << std::endl;
	for (int i = 0; i < outCount; ++i) {
       std::cerr << outData.at(i) << std::endl;
    }
}

//TODO(nebraszka) Invalid pipeline when yield node not provided -> but the errors code is 500 in such a case