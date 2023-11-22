#include <helpers/commonHelpers.hpp>

#include <math/Mat3x4f.hpp>

#if RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

class GraphGetResultTest : public RGLTest
{};

TEST_F(GraphGetResultTest, GetResultsDataWithoutPriorRun)
{
	// Setup
	rgl_node_t pointsFromArray = nullptr;
	rgl_vec3f points = {0};
	rgl_field_t fields = RGL_FIELD_XYZ_VEC3_F32;
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&pointsFromArray, &points, 1, &fields, 1));

	// Test
	EXPECT_RGL_STATUS(rgl_graph_get_result_data(pointsFromArray, RGL_FIELD_XYZ_VEC3_F32, &points), RGL_INVALID_PIPELINE,
	                  "it hasn't been run yet");
}

TEST_F(GraphGetResultTest, GetResultsSizeWithoutPriorRun)
{
	// Setup
	rgl_node_t pointsFromArray = nullptr;
	rgl_vec3f points = {0};
	rgl_field_t fields = RGL_FIELD_XYZ_VEC3_F32;
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&pointsFromArray, &points, 1, &fields, 1));

	// Test
	int32_t count, size;
	EXPECT_RGL_STATUS(rgl_graph_get_result_size(pointsFromArray, RGL_FIELD_XYZ_VEC3_F32, &count, &size), RGL_INVALID_PIPELINE);
}