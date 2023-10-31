#include <helpers/commonHelpers.hpp>

#include <RGLFields.hpp>

#include <api/apiCommon.hpp>
#include <helpers/graphHelpers.hpp>

using namespace ::testing;

// TODO(prybicki): A little refactoring here wouldn't hurt, especially with GraphBuilder

struct SetNodePriority : RGLTest
{
	static constexpr int CHECKS = 32;
	static constexpr double SLEEP = 0.001;
	Vec3f data[1] = {{}};
	rgl_field_t fields[1] = {XYZ_VEC3_F32};

	static bool prioritiesMatch(const std::map<rgl_node_t, int32_t>& expectedPriorities)
	{
		for (auto&& [node, expectedPriority] : expectedPriorities) {
			int32_t outPriority;
			EXPECT_RGL_SUCCESS(rgl_graph_node_get_priority(node, &outPriority));
			if (outPriority != expectedPriority) {
				return false;
			}
		}
		return true;
	}
};

TEST_F(SetNodePriority, TwoBranches)
{
	rgl_node_t fromArray = nullptr;
	double timestampA, timestampB;
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&fromArray, data, 1, fields, 1));

	// Branch A
	rgl_node_t nodeA = nullptr;
	rgl_node_t sleepA = nullptr;
	createOrUpdateNode<RecordTimeNode>(&nodeA);
	createOrUpdateNode<SleepNode>(&sleepA, SLEEP);
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArray, sleepA));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepA, nodeA));

	// Branch B
	rgl_node_t nodeB = nullptr;
	rgl_node_t sleepB = nullptr;
	createOrUpdateNode<RecordTimeNode>(&nodeB);
	createOrUpdateNode<SleepNode>(&sleepB, SLEEP);
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArray, sleepB));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepB, nodeB));

	// Normally, nodes should be executed in the order of adding them
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));
		timestampA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timestampB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		ASSERT_LE(timestampA, timestampB);
	}

	// Setting priority should change order to execute B first
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeB, 1));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));
		timestampA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timestampB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		ASSERT_LE(timestampB, timestampA);
	}

	// Bumping A's priority to greater value than B's should make it execute first again
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeA, 2));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));
		timestampA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timestampB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		ASSERT_LE(timestampA, timestampB);
	}

	// At the end, check if we get expected error when trying to set parent's priority lower than its children
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_node_set_priority(fromArray, 1), "cannot set priority");

	// At the end, priorities should look like this:
	std::map<rgl_node_t, int32_t> expectedPriorities = {
	    {    nodeA, 2},
        {   sleepA, 2},
        {fromArray, 2},
        {    nodeB, 1},
        {   sleepB, 1}
    };
	EXPECT_TRUE(prioritiesMatch(expectedPriorities));
}

TEST_F(SetNodePriority, TwoEntryNodes)
{
	rgl_node_t fromArrayA = nullptr, sleepA = nullptr, nodeA = nullptr;
	rgl_node_t fromArrayB = nullptr, sleepB = nullptr, nodeB = nullptr;
	rgl_node_t merge = nullptr;
	double timestampA, timestampB;

	// Branch A
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&fromArrayA, data, 1, fields, 1));
	createOrUpdateNode<SleepNode>(&sleepA, SLEEP);
	createOrUpdateNode<RecordTimeNode>(&nodeA);
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayA, sleepA));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepA, nodeA));

	// Branch B
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&fromArrayB, data, 1, fields, 1));
	createOrUpdateNode<SleepNode>(&sleepB, SLEEP);
	createOrUpdateNode<RecordTimeNode>(&nodeB);
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArrayB, sleepB));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepB, nodeB));

	// Merge
	ASSERT_RGL_SUCCESS(rgl_node_points_spatial_merge(&merge, fields, 1));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(nodeA, merge));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(nodeB, merge));

	// Without specifying priorities, the ordering of multi-entry graph execution is unspecified.

	// Setting priority should change order to execute B first
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeB, 1));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(merge));
		timestampA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timestampB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		ASSERT_LE(timestampB, timestampA);
	}

	// Bumping A's priority to greater value than B's should make it execute first again
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeA, 2));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(merge));
		timestampA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timestampB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		ASSERT_LE(timestampA, timestampB);
	}

	// At the end, priorities should look like this:
	std::map<rgl_node_t, int32_t> expectedPriorities = {
	    {     nodeA, 2},
        {    sleepA, 2},
        {fromArrayA, 2},
        {     nodeB, 1},
        {    sleepB, 1},
        {fromArrayB, 1}
    };
	EXPECT_TRUE(prioritiesMatch(expectedPriorities));
}

TEST_F(SetNodePriority, AddChild)
{
	double timestampA, timestampB, timestampC;

	rgl_node_t fromArray = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_from_array(&fromArray, data, 1, fields, 1));

	// Branch A, priority 1
	rgl_node_t sleepA = nullptr, nodeA = nullptr;
	createOrUpdateNode<RecordTimeNode>(&nodeA);
	createOrUpdateNode<SleepNode>(&sleepA, SLEEP);
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArray, sleepA));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepA, nodeA));
	EXPECT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeA, 1));

	// Branch B, priority -1
	rgl_node_t sleepB = nullptr, nodeB = nullptr;
	createOrUpdateNode<RecordTimeNode>(&nodeB);
	createOrUpdateNode<SleepNode>(&sleepB, SLEEP);
	{
		EXPECT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeB, -1));
		// Connect child with lower priority, its priority should not propagate to parent
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepB, nodeB));
		std::map<rgl_node_t, int32_t> expectedPriorities = {
		    { nodeB, -1},
            {sleepB,  0}
        };
		EXPECT_TRUE(prioritiesMatch(expectedPriorities));
		// Now explicitly set sleepB priority to -1, to have all branch C on -1
		EXPECT_RGL_SUCCESS(rgl_graph_node_set_priority(sleepB, -1));
	}
	// Branch B still not connected to root

	// Branch C, priority 2
	rgl_node_t sleepC = nullptr, nodeC = nullptr;
	createOrUpdateNode<RecordTimeNode>(&nodeC);
	createOrUpdateNode<SleepNode>(&sleepC, SLEEP);
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(sleepC, nodeC));
	EXPECT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeC, 2));
	// Not yet connected to root

	// Run once without branches B and C
	ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));

	// Connect childs with lower and higher priority
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArray, sleepB));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(fromArray, sleepC));

	// Expect that the nodes will run in expected order - C, A, B
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));
		timestampA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timestampB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		timestampC = dynamic_cast<RecordTimeNode*>(nodeC)->getMeasurement();
		ASSERT_LE(timestampC, timestampA);
		ASSERT_LE(timestampA, timestampB);
	}

	// Verify priorities
	std::map<rgl_node_t, int32_t> expectedPriorities2 = {
	    {    nodeA,  1},
        {   sleepA,  1},
        {fromArray,  2},
        {    nodeB, -1},
        {   sleepB, -1},
        {    nodeC,  2},
        {   sleepC,  2}
    };
	EXPECT_TRUE(prioritiesMatch(expectedPriorities2));
}
