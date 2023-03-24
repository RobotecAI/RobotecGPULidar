#include <random>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <RGLExceptions.hpp>
#include <RGLFields.hpp>


using namespace ::testing;

/**
 * This test is supposed to cover different situations when rgl_graph_add_child is called:
 * - Parent is a correct / incorrect input to the child (onInputValidate() passes or fails)
 * - Parent / child differ of being hot / cold (having GraphContext assigned) (onGraphChange() passes or fails)
 */
struct GraphAddChild : public RGLAutoCleanupTestWithParam<bool>
{
protected:
	GraphAddChild()
	{
		// Prepare a set of test nodes to cover all cases. In some tests, some nodes will remain unused.
		EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&fromArrayA, points, 1, fields, 1));
		EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&fromArrayB, points, 1, fields, 1));
		EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldA, fields, 1));
		EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yieldB, fields, 1));
	}

	void addChild(rgl_node_t parent, rgl_node_t childYield, bool expectAddChildSuccess = true)
	{
		bool correctInput = GetParam();
		if (!correctInput) {
			// Modify child node to make it report invalid (incompatible) input
			EXPECT_RGL_SUCCESS(rgl_node_points_yield(&childYield, wrongFields, 1));
		}

		if (!correctInput) {
			EXPECT_THROW(rgl_graph_node_add_child(parent, childYield), InvalidPipeline);
			SUCCEED(); // Do not proceed when onInputChange() was supposed to fail
		} else {
			if (expectAddChildSuccess) {
				EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(parent, childYield));
			} else {
				// Joining graphs with different contexts should fail
				EXPECT_THROW(rgl_graph_node_add_child(parent, childYield), InvalidPipeline);
			}
		}
	}

	void randomizeParent(rgl_node_t parentFromArray)
	{
		points[0] = {f32(random), f32(random), f32(random)};
	}

	void checkChild(rgl_node_t childYield)
	{
		int32_t count = 0;
		int32_t size = 0;
		rgl_vec3f output[1] = {0};
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(childYield, fields[0], &count, &size));
		ASSERT_EQ(count, 1);
		ASSERT_EQ(size, getFieldSize(fields[0]));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(childYield, fields[0], &output));
		EXPECT_EQ(output[0].value[0], points[0].value[0]);
		EXPECT_EQ(output[0].value[1], points[0].value[1]);
		EXPECT_EQ(output[0].value[2], points[0].value[2]);
	}

protected:
	rgl_field_t fields[1] = {XYZ_F32};
	rgl_field_t wrongFields[1] = {INTENSITY_F32};
	rgl_vec3f points[1] = {{42.0f, 61.5f, 0.14f}};
	rgl_node_t yieldA = nullptr;
	rgl_node_t yieldB = nullptr;
	rgl_node_t fromArrayA = nullptr;
	rgl_node_t fromArrayB = nullptr;
	std::default_random_engine random{42}; // Using random just to get some float sequence, may be repeatable.
	std::uniform_real_distribution<float> f32;
};

// Two separate instantiations for more readable test names.
INSTANTIATE_TEST_SUITE_P(CorrectInput, GraphAddChild, testing::Values(true));
INSTANTIATE_TEST_SUITE_P(WrongInput, GraphAddChild, testing::Values(false));

TEST_P(GraphAddChild, TwoCold)
{
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));

	// Check data propagates when running child
	randomizeParent(fromArrayA);
	EXPECT_RGL_SUCCESS(rgl_graph_run(yieldB));
	checkChild(yieldB);
}

TEST_P(GraphAddChild, HotParentColdChild)
{
	// Make parent hot
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayA));

	// Connect hot parent and cold child
	addChild(fromArrayA, yieldA);

	// Check data propagates when running child
	randomizeParent(fromArrayA);
	EXPECT_RGL_SUCCESS(rgl_graph_run(yieldA));
	checkChild(yieldA);
}

TEST_P(GraphAddChild, ColdParentHotChild)
{
	// Use *B nodes to create separate hot child node (yieldB)
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayB, yieldB));
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayB));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(fromArrayB, yieldB));

	// Connect cold parent and hot child
	addChild(fromArrayA, yieldB);

	// Check data propagates when running child
	randomizeParent(fromArrayA);
	EXPECT_RGL_SUCCESS(rgl_graph_run(yieldB));
	checkChild(yieldB);
}

TEST_P(GraphAddChild, HotParentHotChildSameContext)
{
	// Turn fromArrayA, yieldA into hot, disconnected nodes sharing context
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayA));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(fromArrayA, fromArrayB));

	// Connect hot parent and hot child
	addChild(fromArrayA, fromArrayB);

	// Check data propagates when running child
	randomizeParent(fromArrayA);
	EXPECT_RGL_SUCCESS(rgl_graph_run(yieldA));
	checkChild(yieldA);
}

TEST_P(GraphAddChild, HotParentHotChildDifferentContext)
{
	// Make fromArrayA, yieldA hot
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayA));

	// Make fromArrayB, yieldB hot, disconnect yieldB
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayB, yieldB));
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayB));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(fromArrayB, yieldB));

	// Try connecting yieldB to graph A, expect fail due to different contexts
	addChild(yieldA, yieldB, false);
}
