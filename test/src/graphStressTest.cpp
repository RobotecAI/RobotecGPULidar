#include <random>
#include <gtest/gtest.h>

#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <graph/NodesCore.hpp>

// TODO: Feel free to change naming & organization here.

using namespace ::testing;

struct GraphStress : public RGLAutoCleanupTest {};

static std::random_device randomDevice;
static auto randomSeed = randomDevice();
static std::mt19937 randomGenerator {randomSeed};

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

		setRandomRays();
	}

	void setRandomRays()
	{
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



/**
 * The test runs a graph and concurrently to its execution, it
 * - changes numbers of rays of UseRaysNode
 * - queries output from a subset of random TransformPointsNodes
 * - may call rgl_graph_run while it is still running
 * The test aims to detect any inconsistency that would be caused by race condition bug.
 *
 * TODO: Possible improvements (perhaps separate tests):
 * - When the graph is running, do also scene modifications to stress scene / graph concurrency.
 * - Except modifying just the first node, modify also parameters (transforms) of TransformPointsNodes
 */
TEST_F(GraphStress, Async)
{
	int GRAPH_COUNT = std::thread::hardware_concurrency()  * 2;
	const int GRAPH_SIZE = 128;
	const int GRAPH_QUERIES = 32;
	const int GRAPH_RUNS = 1024;
	const float EPSILON = 1E-6;
	ASSERT_THAT(GRAPH_QUERIES, Lt(GRAPH_SIZE));

	// Optional logging
	// EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_ALL, nullptr, true));

	// Setup scene
	Vec3f cubePosition = {0, 0, 5};
	setupSceneCube(cubePosition);

	// Print random seed for reproducibility
	fmt::print(stderr, "Graph Stress Test random seed: {}\n", randomSeed);

	// Setup random graph
	RandomGraph graph {GRAPH_SIZE};
	ASSERT_FALSE(HasFailure()); // Graph build must have succeeded.

	// Generate random order to query nodes for results
	std::vector<int> checkOrder((graph.transformNodes.size()));
	std::iota(checkOrder.begin(), checkOrder.end(), 0);
	std::shuffle(checkOrder.begin(), checkOrder.end(), randomGenerator);
	checkOrder.resize(GRAPH_QUERIES);

	for (int r = 0; r < GRAPH_RUNS; ++r) {
		// Randomize node index that after querying it, graph will be modified (possibly when still running)
		rgl_node_t updateRaysAfterCheckingNode = graph.transformNodes.at(
		checkOrder.at(std::uniform_int_distribution(0UL, checkOrder.size() - 1)(randomGenerator)));

		// Copy expected values for results (point count and transforms for each node)
		auto expectedPointCount = graph.rayCount.x() * graph.rayCount.y();

		// Save a map of expected transforms for each node
		auto expectedTransform = graph.getCumulativeTransform();

		// Save expected hit point
		Vec3f expectedPointWorld = Vec3f{graph.raysXY.x(), graph.raysXY.y(), cubePosition.z() - 1};

		// Run graph asynchronously
		EXPECT_RGL_SUCCESS(rgl_graph_run(graph.raytraceNode));

		// Check output of nodes in previously generated random order
		for (auto &&checkedNodeIndex: checkOrder) {
			int pointCount = -1, pointSize = -1;

			// Check if output size is not affected by graph modifications
			rgl_node_t checkedNode = graph.transformNodes.at(checkedNodeIndex);
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(checkedNode, RGL_FIELD_XYZ_F32, &pointCount, &pointSize));
			EXPECT_THAT(pointCount, Eq(expectedPointCount));

			// Get results
			std::vector<Vec3f> results(pointCount);
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(checkedNode, RGL_FIELD_XYZ_F32, results.data()));

			// Verify results are correctly transformed
			Vec3f expectedPoint = expectedTransform.at(checkedNode) * expectedPointWorld;
			EXPECT_THAT(results[0].x(), FloatNear(expectedPoint.x(), EPSILON));
			EXPECT_THAT(results[0].y(), FloatNear(expectedPoint.y(), EPSILON));
			EXPECT_THAT(results[0].z(), FloatNear(expectedPoint.z(), EPSILON));

			// All points should be equal
			for (int i = 1; i < results.size(); ++i) {
				EXPECT_THAT(results[i].x(), FloatEq(results[i - 1].x()));
				EXPECT_THAT(results[i].y(), FloatEq(results[i - 1].y()));
				EXPECT_THAT(results[i].z(), FloatEq(results[i - 1].z()));
			}

			// At some point, modify rays while the graph may be still running
			if (checkedNode == updateRaysAfterCheckingNode) {
				graph.setRandomRays();
			}
		}
	}
}
