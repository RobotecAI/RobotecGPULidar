#include <helpers/commonHelpers.hpp>

#include <math/Mat3x4f.hpp>

#if RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

class GraphNodeInputImpactTest : public RGLTest
{};

TEST_F(GraphNodeInputImpactTest, DetectMissingInput)
{
	rgl_node_t compact = nullptr; // Simplest to construct Node requiring Input
	ASSERT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_STATUS(rgl_graph_run(compact), RGL_INVALID_PIPELINE);
}

TEST_F(GraphNodeInputImpactTest, NoInputNodesDoNotRequireInput)
{
	rgl_node_t pointsFromArray = nullptr;
	rgl_vec3f points = {0};
	rgl_field_t fields = RGL_FIELD_XYZ_VEC3_F32;
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&pointsFromArray, &points, 1, &fields, 1));

	// TODO(nebraszka): Test that all other nodes that inherit from INoInputNode can be run with no input
	EXPECT_RGL_SUCCESS(rgl_graph_run(pointsFromArray));
}