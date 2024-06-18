#pragma once

#include <helpers/commonHelpers.hpp>

#include <math/Mat3x4f.hpp>

static std::vector<rgl_mat3x4f> makeLidar3dRays(float fov_x, float fov_y, float resolution_x = 1.0f, float resolution_y = 1.0f)
{
	std::vector<rgl_mat3x4f> rays;

	EXPECT_TRUE(resolution_x > 0.0f);
	EXPECT_TRUE(resolution_y > 0.0f);

	fov_x = std::min(std::max(0.0f, fov_x), 360.0f);
	fov_y = std::min(std::max(0.0f, fov_y), 360.0f);

	float angle_start_x = -1 * fov_x / 2.0f;
	float angle_start_y = -1 * fov_y / 2.0f;

	for (float add_x = 0.0f; add_x <= fov_x; add_x += resolution_x) {
		float rot_x = angle_start_x + add_x;
		for (float add_y = 0.0f; add_y <= fov_y; add_y += resolution_y) {
			float rot_y = angle_start_y + add_y;
			rays.emplace_back(Mat3x4f::rotationDeg(rot_x, rot_y, 0.0).toRGL());
		}
	}

	return rays;
}

static std::vector<rgl_mat3x4f> makeGridOfParallelRays(Vec2f minCoord, Vec2f maxCoord, Vec2i count)
{
	std::vector<rgl_mat3x4f> rays;
	EXPECT_TRUE(count.x() > 0);
	EXPECT_TRUE(count.y() > 0);

	float spacingY = (maxCoord.y() - minCoord.y()) / (count.y() + 1);
	float spacingX = (maxCoord.x() - minCoord.x()) / (count.x() + 1);
	for (int y = 1; y <= count.y(); ++y) {
		for (int x = 1; x <= count.x(); ++x) {
			Vec3f origin{minCoord.x() + spacingX * static_cast<float>(x), minCoord.y() + spacingY * static_cast<float>(y),
			             0.0f};
			rays.emplace_back(Mat3x4f::translation(origin.x(), origin.y(), origin.z()).toRGL());
		}
	}
	return rays;
}


#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>

#include <RGLFields.hpp>

/**
* @brief Auxiliary lidar graph for imaging entities on the scene
*/
static rgl_node_t constructCameraGraph(const rgl_mat3x4f& cameraPose, const char* cameraName = "camera")
{
	const std::vector<rgl_field_t> fields{XYZ_VEC3_F32};
	const std::vector<rgl_mat3x4f> cameraRayTf = makeLidar3dRays(360.0f, 180.0f, 0.5f, 0.5f);
	rgl_node_t cameraRays = nullptr, cameraTransform = nullptr, cameraRaytrace = nullptr, cameraFormat = nullptr,
	           cameraPublish = nullptr;

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&cameraRays, cameraRayTf.data(), cameraRayTf.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&cameraTransform, &cameraPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&cameraRaytrace, nullptr));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&cameraFormat, fields.data(), fields.size()));
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&cameraPublish, cameraName, "world"));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraRays, cameraTransform));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraTransform, cameraRaytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraRaytrace, cameraFormat));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(cameraFormat, cameraPublish));

	return cameraRays;
}
#endif