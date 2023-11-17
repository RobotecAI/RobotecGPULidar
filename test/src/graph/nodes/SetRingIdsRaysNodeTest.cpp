#include <helpers/commonHelpers.hpp>

#include <math/Mat3x4f.hpp>

class SetRingIdsNodeTest : public RGLTestWithParam<int>
{
protected:
	rgl_node_t setRingIdsNode, rayNode;
	std::vector<int> ringIds;
	std::vector<rgl_mat3x4f> rays;

	void initializeRingNodeAndIds(int idsCount)
	{
		setRingIdsNode = nullptr;
		ringIds.resize(idsCount);
		std::iota(ringIds.begin(), ringIds.end(), 0);
	}

	void initializeRaysAndRaysNode(int rayCount)
	{
		rayNode = nullptr;
		rays.clear();
		rays.reserve(rayCount);
		for (int i = 0; i < rayCount; i++) {
			rays.emplace_back(
			    Mat3x4f::translation(static_cast<float>(i), static_cast<float>(i + 1), static_cast<float>(i + 2)).toRGL());
		}
	}
};

INSTANTIATE_TEST_SUITE_P(SetRingIdsNodeTests, SetRingIdsNodeTest, testing::Values(1, 10, 100, maxGPUCoresTestCount),
                         [](const auto& info) { return "idsCount_" + std::to_string(info.param); });

TEST_F(SetRingIdsNodeTest, invalid_argument_node)
{
	initializeRingNodeAndIds(1);
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(nullptr, ringIds.data(), ringIds.size()), "node != nullptr");
}

TEST_F(SetRingIdsNodeTest, invalid_argument_ring_ids)
{
	initializeRingNodeAndIds(1);
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&setRingIdsNode, nullptr, ringIds.size()), "ring_ids != nullptr");
}

TEST_F(SetRingIdsNodeTest, invalid_argument_ring_ids_count)
{
	initializeRingNodeAndIds(1);
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), 0), "ring_ids_count > 0");
}

TEST_F(SetRingIdsNodeTest, valid_arguments)
{
	initializeRingNodeAndIds(1);
	ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
	EXPECT_THAT(setRingIdsNode, testing::NotNull());

	// If (*setRingIdsNode) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
}

TEST_P(SetRingIdsNodeTest, invalid_pipeline_less_rays_than_ring_ids)
{
	int32_t idsCount = GetParam();
	if (idsCount / 2 == 0) {
		return;
	}

	//// Incorrect number of ring ids passed to the rgl_node_rays_set_ring_ids ////
	initializeRingNodeAndIds(idsCount);
	initializeRaysAndRaysNode(idsCount / 2);
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, setRingIdsNode));

	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(setRingIdsNode), "ring ids doesn't match number of rays");

	//// Changed number of rays between graph runs ////
	// Initialize and run valid pipeline
	initializeRingNodeAndIds(idsCount);
	initializeRaysAndRaysNode(idsCount);
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, setRingIdsNode));
	EXPECT_RGL_SUCCESS(rgl_graph_run(setRingIdsNode));

	// Make pipeline invalid
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size() / 2));
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(setRingIdsNode), "ring ids doesn't match number of rays");
}

TEST_P(SetRingIdsNodeTest, valid_pipeline_equal_number_of_rays_and_ring_ids)
{
	int32_t idsCount = GetParam();

	initializeRingNodeAndIds(idsCount);
	initializeRaysAndRaysNode(idsCount);
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
	ASSERT_THAT(setRingIdsNode, testing::NotNull());
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, setRingIdsNode));

	EXPECT_RGL_SUCCESS(rgl_graph_run(setRingIdsNode));
}
