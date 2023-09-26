
#include <gtest/gtest.h>
#include <RGLFields.hpp>
#include <utils.hpp>
#include <nodes.hpp>

#include <api/apiCommon.hpp>

using namespace ::testing;

struct SetNodePriority : RGLTest
{
	static constexpr int CHECKS = 32;
	static constexpr double SLEEP = 0.001;
	Vec3f data[1] = {{}};
	rgl_field_t fields[1] = {XYZ_F32};
};

// TODO(nebraszka): add the following tests
// Priority propagates through parents
// Cannot set priority lower than max of children
// Adding a child with a high priority has the same effect as adding a child and settings its high priority
// Also, existing tests would benefit from refactor.

TEST_F(SetNodePriority, Basic)
{
	rgl_node_t fromArray = nullptr;
	double timeA, timeB;
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
		timeA = dynamic_cast<RecordTimeNode*>(nodeA)->getMeasurement();
		timeB = dynamic_cast<RecordTimeNode*>(nodeB)->getMeasurement();
		ASSERT_LE(timeA, timeB);
	}

	// Setting priority should change order to execute B first
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeB, 1));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));
		timeA = dynamic_cast<RecordTimeNode *>(nodeA)->getMeasurement();
		timeB = dynamic_cast<RecordTimeNode *>(nodeB)->getMeasurement();
		ASSERT_LE(timeB, timeA);
	}

	// Bumping A's priority to greater value than B's should make it execute first again
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeA, 2));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(fromArray));
		timeA = dynamic_cast<RecordTimeNode *>(nodeA)->getMeasurement();
		timeB = dynamic_cast<RecordTimeNode *>(nodeB)->getMeasurement();
		ASSERT_LE(timeA, timeB);
	}

	// At the end, check if we get expected error when trying to set parent's priority lower than its children
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_node_set_priority(fromArray, 1), "cannot set priority");
}

TEST_F(SetNodePriority, TwoEntryNodes)
{
	rgl_node_t fromArrayA = nullptr, sleepA = nullptr, nodeA = nullptr;
	rgl_node_t fromArrayB = nullptr, sleepB = nullptr, nodeB = nullptr;
	rgl_node_t merge = nullptr;
	double timeA, timeB;

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
		timeA = dynamic_cast<RecordTimeNode *>(nodeA)->getMeasurement();
		timeB = dynamic_cast<RecordTimeNode *>(nodeB)->getMeasurement();
		ASSERT_LE(timeB, timeA);
	}

	// Bumping A's priority to greater value than B's should make it execute first again
	ASSERT_RGL_SUCCESS(rgl_graph_node_set_priority(nodeA, 2));
	for (int i = 0; i < CHECKS; ++i) {
		ASSERT_RGL_SUCCESS(rgl_graph_run(merge));
		timeA = dynamic_cast<RecordTimeNode *>(nodeA)->getMeasurement();
		timeB = dynamic_cast<RecordTimeNode *>(nodeB)->getMeasurement();
		ASSERT_LE(timeA, timeB);
	}
}