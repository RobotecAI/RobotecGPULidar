#pragma once

#include <math/Mat3x4f.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

static std::vector<rgl_mat3x4f> makeLidar3dRays(float fov_x, float fov_y,
                                                float resolution_x = 1.0f,
                                                float resolution_y = 1.0f)
{
    std::vector<rgl_mat3x4f> rays;

    EXPECT_TRUE(resolution_x > 0.0f);
    EXPECT_TRUE(resolution_y > 0.0f);

    fov_x = std::min(std::max(0.0f, fov_x), 360.0f);
    fov_y = std::min(std::max(0.0f, fov_y), 360.0f);

    float angle_start_x = -1 * fov_x / 2.0f;
    float angle_start_y = -1 * fov_y / 2.0f;

    for (float add_x = 0.0f; add_x <= fov_x; add_x += resolution_x)
    {
        float rot_x = angle_start_x + add_x;
        for (float add_y = 0.0f; add_y <= fov_y; add_y += resolution_y)
        {
            float rot_y = angle_start_y + add_y;
            rays.push_back(Mat3x4f::rotation(rot_x, rot_y, 0.0).toRGL());
        }
    }

    return rays;
}

static std::vector<rgl_mat3x4f> makeGridOfParallelRays(Vec2f minCoord, Vec2f maxCoord, Vec2i count)
{
	std::vector<rgl_mat3x4f> rays;
	EXPECT_TRUE(count.x() > 0);
	EXPECT_TRUE(count.y() > 0);

	float dx = (maxCoord.x() - minCoord.x()) / count.x();
	float dy = (maxCoord.y() - minCoord.y()) / count.y();
	for (int y = 0; y < count.y(); ++y) {
		for (int x = 0; x < count.x(); ++x) {
			Vec3f origin {
				minCoord.x() + dx * static_cast<float>(x),
				minCoord.y() + dy * static_cast<float>(y),
				0.0f
			};
			rays.emplace_back(Mat3x4f::translation(origin.x(), origin.y(), origin.z()).toRGL());
		}
	}
	return rays;
}