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
 * This test uses a very limited set of nodes, since the tested functions are implemented by base class Node
 * - FromArrayPointsNode at root to inject some data into a graph
 * - YieldPoints everywhere else; allows to change its required fields and thus to trigger invalid input detection
 */
struct GraphAddChild : public RGLTestWithParam<bool>
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

	// Returns true if the test should succeed immediately
	bool addChild(rgl_node_t parent, rgl_node_t childYield, bool expectAddChildSuccess = true)
	{
		bool correctInput = GetParam();
		if (!correctInput) {
			// Modify child node to make it report invalid (incompatible) input
			EXPECT_RGL_SUCCESS(rgl_node_points_yield(&childYield, wrongFields, 1));
			EXPECT_RGL_STATUS(rgl_graph_node_add_child(parent, childYield), RGL_INVALID_PIPELINE);
			return true; // Do not proceed when onInputChangeImpl() was supposed to fail
		}

		if (expectAddChildSuccess) {
			EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(parent, childYield));
		} else {
			// Joining graphs with different contexts should fail
			EXPECT_RGL_STATUS(rgl_graph_node_add_child(parent, childYield), RGL_INVALID_PIPELINE);
		}
		return false;
	}

	bool caseWithInvalidInput() { return !GetParam(); }

	void updateFromArray(rgl_node_t fromArray)
	{
		points[0] = {f32(random), f32(random), f32(random)};
		EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&fromArray, points, 1, fields, 1));
	}

	void updateYield(rgl_node_t yield)
	{
		if (caseWithInvalidInput()) {
			EXPECT_RGL_SUCCESS(rgl_node_points_yield(&yield, wrongFields, 1));
		}
	}

	void tryRunAndCheck(rgl_node_t yield)
	{
		if (caseWithInvalidInput()) {
			EXPECT_RGL_STATUS(rgl_graph_run(yield), RGL_INVALID_PIPELINE);
			return;
		}
		int32_t count = 0;
		int32_t size = 0;
		rgl_vec3f output[1] = {0};
		EXPECT_RGL_SUCCESS(rgl_graph_run(yield));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(yield, fields[0], &count, &size));
		ASSERT_EQ(count, 1);
		ASSERT_EQ(size, getFieldSize(fields[0]));
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(yield, fields[0], &output));
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
	std::default_random_engine random{42}; // Randomness is not needed, using it just to get some float sequence.
	std::uniform_real_distribution<float> f32;
};

// Two separate instantiations for more readable test names.
INSTANTIATE_TEST_SUITE_P(CorrectInput, GraphAddChild, testing::Values(true));
INSTANTIATE_TEST_SUITE_P(WrongInput, GraphAddChild, testing::Values(false));

#define SUCCEED_IF_TRUE(statement) if (statement) { return; }
TEST_P(GraphAddChild, ColdChildColdParent)
{
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));

	// Check data propagates when running child
	updateFromArray(fromArrayA);
	updateYield(yieldA);
	tryRunAndCheck(yieldA);
}

TEST_P(GraphAddChild, HotParentColdChild)
{
	// Make parent hot
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayA));

	// Connect hot parent and cold child
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));

	// Check data propagates when running child
	updateFromArray(fromArrayA);
	updateYield(yieldA);
	tryRunAndCheck(yieldA);
}

TEST_P(GraphAddChild, ColdParentHotChild)
{
	// Use *B nodes to create separate hot child node (yieldB)
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayB, yieldB));
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayB));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(fromArrayB, yieldB));

	// Connect cold parent and hot child
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldB));

	// Check data propagates when running child
	updateFromArray(fromArrayA);
	updateYield(yieldB);
	tryRunAndCheck(yieldB);
}

TEST_P(GraphAddChild, HotParentHotChildSameContext)
{
	// Turn fromArrayA, yieldA into hot, disconnected nodes sharing context
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));
	EXPECT_RGL_SUCCESS(rgl_graph_run(fromArrayA));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(fromArrayA, yieldA));

	// Connect hot parent and hot child
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, yieldA));

	// Check data propagates when running child
	updateFromArray(fromArrayA);
	updateYield(yieldA);
	tryRunAndCheck(yieldA);
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

	// Connect two hot nodes from different graph run contexts
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(yieldA, yieldB));

	updateFromArray(fromArrayA);
	updateYield(yieldB);
	tryRunAndCheck(yieldB);
}
