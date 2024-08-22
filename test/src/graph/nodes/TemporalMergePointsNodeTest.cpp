#include <helpers/commonHelpers.hpp>
#include <helpers/sceneHelpers.hpp>
#include <helpers/lidarHelpers.hpp>

#include <math/Mat3x4f.hpp>
#include <Logger.hpp>

#if RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif

class TemporalMergePointsNodeTest : public RGLTest
{
protected:
	std::vector<rgl_field_t> fields;
	rgl_node_t temporalMergePointsNode;

	TemporalMergePointsNodeTest()
	{
		// TODO(nebraszka): Parameterize the test to take a permutation of the set of all fields
		fields = {RGL_FIELD_INTENSITY_F32, RGL_FIELD_AZIMUTH_F32};
		temporalMergePointsNode = nullptr;
	}
};

TEST_F(TemporalMergePointsNodeTest, invalid_argument_node)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(nullptr, fields.data(), fields.size()), "node != nullptr");
}

TEST_F(TemporalMergePointsNodeTest, invalid_argument_fields)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(&temporalMergePointsNode, nullptr, fields.size()),
	                            "fields != nullptr");
}

TEST_F(TemporalMergePointsNodeTest, invalid_argument_field_count)
{
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(&temporalMergePointsNode, fields.data(), 0), "field_count > 0");
}

TEST_F(TemporalMergePointsNodeTest, valid_arguments)
{
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMergePointsNode, fields.data(), fields.size()));
	ASSERT_THAT(temporalMergePointsNode, testing::NotNull());

	// If (*raysFromMatNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMergePointsNode, fields.data(), fields.size()));
}

TEST_F(TemporalMergePointsNodeTest, temporal_merge)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_transform(entity, &entityPoseTf));

	rgl_node_t useRays = nullptr, raytrace = nullptr, lidarPose = nullptr, compact = nullptr, transformPts = nullptr;
	rgl_node_t temporalMerge = nullptr;
	std::vector<rgl_field_t> tMergeFields = {RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32};

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact_by_field(&compact, RGL_FIELD_IS_HIT_I32));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, temporalMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Change transform for the next raytrace
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateYTf));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
#if RGL_BUILD_PCL_EXTENSION
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(temporalMerge, "two_boxes_temporal_merge.pcd"));
#else
	RGL_WARN("RGL compiled without PCL extension. Tests will not save PCD!");
#endif
}

/**
 * Temporal merging can lead to exceptionally high memory consumption.
 * In majority of cases CPU memory is much larger than GPU memory,
 * therefore TemporalMergeNode should use by default CPU memory.
 *
 * This test uses CUDA API to get current total memory usage of the GPU,
 * which makes this test inevitably prone to random fluctuations.
 *
 * In the future this might fixed by wrapping all CUDA allocations and exposing API to internally track memory usage.
 *
 * In RGL API we could give user control over which memory is used by this node.
 * However, this raises question, why not allow it for all nodes, which
 * summons another jungle of complexity, which I'd prefer to avoid until it is really necessary.
 */
TEST_F(TemporalMergePointsNodeTest, temporal_merge_uses_host_memory)
{
	const int32_t STEPS = 4;
	const int32_t BYTES_PER_STEP = (32 * 1024 * 1024);
	const int32_t allowedAllocatedBytes = (32 * 1024 * 1024); // Allow fluctuations much smaller than Node's memory needs.

	// Graph variables
	rgl_node_t usePoints = nullptr;
	rgl_node_t temporalMerge = nullptr;
	rgl_field_t fields[] = {RGL_FIELD_RETURN_TYPE_U8};
	int32_t fieldCount = ARRAY_SIZE(fields);
	std::vector<char> data(BYTES_PER_STEP);

	// Setup graph
	rgl_node_points_from_array(&usePoints, data.data(), BYTES_PER_STEP, fields, fieldCount);
	rgl_node_points_temporal_merge(&temporalMerge, fields, fieldCount);
	rgl_graph_node_add_child(usePoints, temporalMerge);

	int64_t freeMemAfterPrevStep = 0;
	int64_t allocatedBytes = 0;
	CHECK_CUDA(cudaMemGetInfo(reinterpret_cast<size_t*>(&freeMemAfterPrevStep), nullptr));
	for (int i = 0; i < STEPS; ++i) {
		rgl_graph_run(usePoints);
		int64_t currentFreeMem = 0;
		CHECK_CUDA(cudaMemGetInfo(reinterpret_cast<size_t*>(&currentFreeMem), nullptr));
		allocatedBytes += (freeMemAfterPrevStep - currentFreeMem);
		freeMemAfterPrevStep = currentFreeMem;
	}

	if (allocatedBytes > allowedAllocatedBytes) {
		FAIL() << fmt::format("TemporalMergeNode seems to allocate GPU memory (allocated {} b)", allocatedBytes);
	}
}