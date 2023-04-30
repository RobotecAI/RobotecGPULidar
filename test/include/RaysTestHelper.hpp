#include <math/Mat3x4f.hpp>

struct RGLRaysTestHelper {

    static std::vector<rgl_mat3x4f> GenerateRays(int numRays)
    {
        std::vector<rgl_mat3x4f> rays;

        rays.reserve(numRays);
        for (int i = 0; i < numRays; i++) {
            rays.emplace_back(Mat3x4f::scale((float)i, (float)i+1, (float)i+2).toRGL());
        }
        return rays;
    }
};