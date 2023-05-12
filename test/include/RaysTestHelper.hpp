#include <utils.hpp>

struct RGLRaysNodeTestHelper {

    std::vector<rgl_mat3x4f> inRays;

    static std::vector<rgl_mat3x4f> generateRays(int numRays)
    {
        std::vector<rgl_mat3x4f> rays;

        rays.reserve(numRays);
        for (int i = 0; i < numRays; i++) {
            rays.emplace_back(Mat3x4f::scale((float)i, (float)i+1, (float)i+2).toRGL());
        }
        return rays;
    }
    
protected:
    rgl_node_t useRaysNode = nullptr;

    void createTestUseRaysNode(int numRays)
    {
        inRays = generateRays(numRays);

        EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRaysNode, inRays.data(), inRays.size()));
        ASSERT_THAT(useRaysNode, testing::NotNull());
    }
};