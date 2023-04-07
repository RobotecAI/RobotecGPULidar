#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <math/Mat3x4f.hpp>

class SetRingIdsNodeTest : public RGLTestWithParam<int> {
protected:
    rgl_node_t setRingIdsNode, rayNode;
    std::vector<int> ringIds;
    std::vector<rgl_mat3x4f> rays;

    static std::vector<int> GenerateRingIds(int numRingIds)
    {
        std::vector<int> points(numRingIds);
        std::iota(points.begin(), points.end(), 0);
        return points;
    }

    static std::vector<rgl_mat3x4f> GenerateRays(int numRays)
    {
        std::vector<rgl_mat3x4f> rays;
        struct Mat3x4f ray = Mat3x4f::identity();

        for (int i = 0; i < numRays; i++) {
            rays.emplace_back(ray.scale(i, i+1, i+2).toRGL());
        }
        return rays;
    }

    void initializeRingNodeAndIds(int idsCount=1)
    {
        setRingIdsNode = nullptr;
        ringIds = GenerateRingIds(idsCount);
    }

    void initializeRaysAndRaysNode(int rayCount)
    {
        rayNode = nullptr;
        rays = GenerateRays(rayCount);
    }
};

 INSTANTIATE_TEST_SUITE_P(
     SetRingIdsNodeTests, SetRingIdsNodeTest,
     testing::Values(1, 10, 100, maxGPUCoresTestCount),
     [](const auto& info) {
         return "idsCount_" + std::to_string(info.param);
     });

 TEST_F(SetRingIdsNodeTest, invalid_arguments)
 {
    initializeRingNodeAndIds();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(nullptr, ringIds.data(), ringIds.size()), "node != nullptr");

    initializeRingNodeAndIds();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&setRingIdsNode, nullptr, ringIds.size()), "ring_ids != nullptr");

    initializeRingNodeAndIds();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), 0), "ring_ids_count > 0");
 }

 TEST_F(SetRingIdsNodeTest, valid_arguments)
 {
    initializeRingNodeAndIds();
    ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
    EXPECT_THAT(setRingIdsNode, testing::NotNull());

    // If (*setRingIdsNode) != nullptr
    initializeRingNodeAndIds();
    EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
 }

 TEST_P(SetRingIdsNodeTest, invalid_pipeline_less_rays_than_ring_ids)
 {
    int32_t idsCount = GetParam();
    if(idsCount/2 == 0){ return; };

    initializeRingNodeAndIds(idsCount);
    initializeRaysAndRaysNode(idsCount/2);
    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
    ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, setRingIdsNode));

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
