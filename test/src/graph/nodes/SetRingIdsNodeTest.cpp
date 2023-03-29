#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class SetRingIdsNodeTest : public RGLTestWithParam<int> {
protected:
    static std::vector<int> GenerateRingIds(int count)
    {
        std::vector<int> points;
        for (int i = 0; i < count; ++i) {
            points.emplace_back(i);
        }
        return points;
    }

    static std::vector<rgl_mat3x4f> GenerateRays(int count)
    {
        std::vector<rgl_mat3x4f> rays;
        for (int i = 0; i < count; i++) {
            rays.emplace_back(
                rgl_mat3x4f { .value = {
                                  (float)i + 1, 0, 0, 0,
                                  0, (float)i + 1, 0, 0,
                                  0, 0, (float)i + 1, 0 } });
        }
        return rays;
    }
};

INSTANTIATE_TEST_SUITE_P(
    SetRingIdsNodeTests, SetRingIdsNodeTest,
    Values(2, 5, 10, 100),
    [](const auto& info) {
        return "idsCount_" + std::to_string(info.param);
    });

TEST_P(SetRingIdsNodeTest, invalid_arguments)
{
    rgl_node_t rayNode = nullptr, setRingIdsNode = nullptr;
    int32_t idsCount = GetParam();
    auto ringIds = GenerateRingIds(idsCount);
    auto rays = GenerateRays(idsCount / 2);

    auto initializeArgumentsLambda = [&rayNode, &idsCount, &ringIds]() {
        rayNode = nullptr;
        idsCount = SetRingIdsNodeTest::GetParam();
        ringIds = SetRingIdsNodeTest::GenerateRingIds(idsCount);
    };

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(nullptr, nullptr, 0), "node != nullptr");

    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&rayNode, nullptr, 0), "ring_ids != nullptr");

    initializeArgumentsLambda();
    EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&rayNode, ringIds.data(), 0), "ring_ids_count > 0");

    initializeArgumentsLambda();
    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
    ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, setRingIdsNode));

    EXPECT_RGL_INVALID_PIPELINE(rgl_graph_run(setRingIdsNode), "ring ids doesn't match number of rays");
}

TEST_P(SetRingIdsNodeTest, valid_arguments)
{
    rgl_node_t rayNode = nullptr, setRingIdsNode = nullptr;
    int32_t idsCount = GetParam();
    auto ringIds = GenerateRingIds(idsCount);
    auto rays = GenerateRays(idsCount);

    ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&rayNode, rays.data(), rays.size()));
    ASSERT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&setRingIdsNode, ringIds.data(), ringIds.size()));
    ASSERT_THAT(setRingIdsNode, NotNull());
    ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(rayNode, setRingIdsNode));

    EXPECT_RGL_SUCCESS(rgl_graph_run(setRingIdsNode));
}
