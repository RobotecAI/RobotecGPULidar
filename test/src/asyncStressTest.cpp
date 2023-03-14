#include <random>
#include <gtest/gtest.h>

#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <graph/NodesCore.hpp>

struct GraphStress : public RGLTest {};
const Vec3f cubePosition = {0, 0, 5};
const Vec3f cubeDims = {2, 2, 2};

static std::random_device randomDevice;
static auto randomSeed = randomDevice();
static std::mt19937 randomGenerator {randomSeed};

// randomizeIndices(10,3): 9, 0, 5
std::vector<int> randomizeIndices(int containerSize, int firstN=0) {
	std::vector<int> indices;
	indices.resize(containerSize);
	std::iota(indices.begin(), indices.end(), 0);
	std::shuffle(indices.begin(), indices.end(), randomGenerator);
	if (firstN > 0) {
		indices.resize(firstN);
	}
	return indices;
}

/**
 * Helper structure that builds a graph with a minimal linear part and then a random subtree of TransformPointsNodes.
 */
struct RandomGraph
{
	rgl_node_t useRaysNode = nullptr;
	rgl_node_t raytraceNode = nullptr;
	rgl_node_t compactNode = nullptr;
	rgl_node_t parentTransformNode = nullptr;

	std::vector<rgl_node_t> transformNodes;
	std::vector<rgl_mat3x4f> raysCurrent;
	std::vector<rgl_mat3x4f> raysNext;

	Vec2f raysXY;
	Vec2i rayCount;

	RandomGraph(int wildSubtreeNodeCount)
	{
		// Build minimal linear part
		rgl_mat3x4f dummyRay = Mat3x4f::identity().toRGL();
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, &dummyRay, 1));
		EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr, 100.0f));
		EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));

		rgl_mat3x4f identity = Mat3x4f::identity().toRGL();

		EXPECT_RGL_SUCCESS(rgl_node_points_transform(&parentTransformNode, &identity));
		transformNodes.push_back(parentTransformNode);

		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRaysNode, raytraceNode));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, parentTransformNode));

		// Build random subtree using TransformPointsNodes

		for (int i = 0; i < wildSubtreeNodeCount; ++i) {
			std::uniform_int_distribution<size_t> randomIndex{0UL, transformNodes.size() - 1};
			std::uniform_real_distribution<float> randomFloat{-1, 1};
			size_t parentIndex = randomIndex(randomGenerator);
			rgl_node_t parentNode = transformNodes.at(parentIndex);
			Mat3x4f childTransform = Mat3x4f::translation(
				randomFloat(randomGenerator),
				randomFloat(randomGenerator),
				randomFloat(randomGenerator)
			);

			rgl_node_t childNode = nullptr;
			rgl_mat3x4f childTransformRGL = childTransform.toRGL();
			EXPECT_RGL_SUCCESS(rgl_node_points_transform(&childNode, &childTransformRGL));
			EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(parentNode, childNode));
			transformNodes.push_back(childNode);
		}

		generateRandomRays();
	}

	// Function to re-randomize
	void generateRandomRays()
	{
		// We want rays to hit the cube and do not bother with edge cases
		std::uniform_real_distribution<float> randomFloat {-0.9, 0.9};
		std::uniform_int_distribution<size_t> randomInt {1, 10};
		raysXY = { randomFloat(randomGenerator), randomFloat(randomGenerator) };
		rayCount = { randomInt(randomGenerator), randomInt(randomGenerator) };
		std::vector<rgl_mat3x4f> rays = makeGridOfParallelRays(raysXY, raysXY, {rayCount.x(), rayCount.y()});
		EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, rays.data(), rays.size()));
	}

	std::unordered_map<rgl_node_t, Mat3x4f> getCumulativeTransform() const
	{
		std::unordered_map<rgl_node_t, Mat3x4f> result;
		std::function<void(rgl_node_t, Mat3x4f)> dfs = [&](rgl_node_t currentBase, Mat3x4f parentCumulative) {
			auto* currentTyped = dynamic_cast<TransformPointsNode*>(currentBase);
			Mat3x4f currentCumulative = currentTyped->getTransform() * parentCumulative;
			result.insert({currentBase, currentCumulative});
			for(auto&& childBase : currentBase->getOutputs()) {
				if (!result.contains(childBase.get())) {
					dfs(childBase.get(), currentCumulative);
				}
			}
		};
		dfs(parentTransformNode, Mat3x4f::identity());
		return result;
	}
};

/* Helper structure for data associated with a single graph and a single run-modify-check cycle */
struct GraphRun {
	int expectedPointCount;
	Vec3f expectedPointWorld;
	rgl_node_t updateRaysAfterCheckingNode;
	std::unordered_map<rgl_node_t, Mat3x4f> expectedTransform;
	std::vector<int> nodeCheckOrder;
	int nextNodeCheckOrderIndex;
};

/**
 * The test runs a graph and concurrently to its execution, it
 * - changes numbers of rays of UseRaysNode
 * - queries output from a subset of random TransformPointsNodes
 * - may call rgl_graph_run while it is still running
 * The test aims to detect any inconsistency that would be caused by race condition bug.
 *
 * TODO: Possible improvements:
 * - When the graph is running, do also scene modifications to stress scene / graph concurrency.
 *
 * - Except modifying just the first node, modify also parameters (transforms) of TransformPointsNodes
 *   - May deteriorate testing concurrency, because requires a relatively long update of cumulative transform map.
 */
TEST_F(GraphStress, Async)
{
	using namespace testing;

	// Config
	int GRAPH_COUNT = 16;  // Number of graphs executed in parallel
	const int GRAPH_SIZE = 1024;  // Number of TransformPointsNodes in each graph
	const int GRAPH_QUERIES = 32;  // How many nodes are checked for results (should be a subset)
	const int GRAPH_EPOCHS = 32;  // How many times all graphs are executed, changed and queried
	const float EPSILON = 2 * 1E-6;  // Tolerance for GPU-CPU matrix multiplication differences
	ASSERT_THAT(GRAPH_QUERIES, Lt(GRAPH_SIZE));

	// Optional logging
	// EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_ALL, nullptr, true));

	setupSceneCube(cubePosition);

	// Print random seed for reproducibility
	fmt::print(stderr, "Graph Stress Test random seed: {}\n", randomSeed);

	// Setup random graphs and helper structures
	std::vector<RandomGraph> graphs;
	std::vector<GraphRun> runs;
	runs.resize(GRAPH_COUNT);
	for (int g = 0; g < GRAPH_COUNT; ++g) {
		graphs.emplace_back(GRAPH_SIZE);
	}

	// TODO: the outer loop (GRAPH_EPOCHS) could be replaced using gtest_repeat
	// TODO: however, it would need external definition of GRAPH_EPOCHS, python script?
	for (int e = 0; e < GRAPH_EPOCHS; ++e) {
		// Randomize graph check order
		std::vector<int> graphCheckOrder = randomizeIndices(GRAPH_COUNT);

		// Prepare graph-run associated - node check order, when apply changes, and expected*
		for (int g = 0; g < GRAPH_COUNT; ++g) {
			auto& run = runs.at(g);
			auto& graph = graphs.at(g);
			auto checksBeforeChange = std::uniform_int_distribution(0, GRAPH_QUERIES - 1)(randomGenerator);
			run.nodeCheckOrder = randomizeIndices(GRAPH_SIZE, GRAPH_QUERIES);
			run.updateRaysAfterCheckingNode = graph.transformNodes.at(run.nodeCheckOrder.at(checksBeforeChange));
			run.expectedPointCount = graph.rayCount.x() * graph.rayCount.y();
			run.expectedTransform = graph.getCumulativeTransform();
			run.expectedPointWorld = Vec3f{graph.raysXY.x(), graph.raysXY.y(), cubePosition.z() - 1};
			run.nextNodeCheckOrderIndex = 0;
		}

		// Run all graphs at once
		for (int g = 0; g < GRAPH_COUNT; ++g) {
			EXPECT_RGL_SUCCESS(rgl_graph_run(graphs.at(g).useRaysNode));
		}
		ASSERT_FALSE(HasFailure());

		// Iterate over graphs and check a single node at a time.
		// Repeat until all graphs are checked (each node from run.nodeCheckOrder is checked)
		bool allGraphsChecked = false;
		while (!allGraphsChecked)
		{
			allGraphsChecked = true; // This will be and-ed with is-checked for all graphs

			// Check a single node from each graph
			for (auto&& g : graphCheckOrder) {
				// Useful aliases
				auto& graph = graphs.at(g);
				auto& run = runs.at(g);
				auto& checkedNode = graph.transformNodes.at(run.nodeCheckOrder.at(run.nextNodeCheckOrderIndex));

				// Exit if all nodes in this graph were checked
				bool allNodesChecked = run.nextNodeCheckOrderIndex == run.nodeCheckOrder.size();
				if (allNodesChecked) {
					allGraphsChecked = allGraphsChecked && allNodesChecked;
					continue;
				}

				// Check if output size is not affected by graph modifications
				int pointCount = -1, pointSize = -1;
				EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(checkedNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));
				EXPECT_EQ(pointCount,run.expectedPointCount);

				// Get results
				std::vector<Vec3f> results(pointCount);
				EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(checkedNode, RGL_FIELD_XYZ_F32, results.data()));

				// Verify results are correctly transformed
				Vec3f expectedPoint = run.expectedTransform.at(checkedNode) * run.expectedPointWorld;
				EXPECT_NEAR(results.at(0).x(), expectedPoint.x(), EPSILON);
				EXPECT_NEAR(results.at(0).y(), expectedPoint.y(), EPSILON);
				EXPECT_NEAR(results.at(0).z(), expectedPoint.z(), EPSILON);

				// All points should be equal
				for (int i = 1; i < results.size(); ++i) {
					EXPECT_EQ(results.at(i).x(), results.at(i - 1).x());
					EXPECT_EQ(results.at(i).y(), results.at(i - 1).y());
					EXPECT_EQ(results.at(i).z(), results.at(i - 1).z());
				}

				// At some point, modify rays while the graph may be still running
				if (checkedNode == run.updateRaysAfterCheckingNode) {
					graph.generateRandomRays();
				}

				run.nextNodeCheckOrderIndex += 1;
			}
		}
	}
}
