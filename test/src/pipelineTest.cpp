#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>

#include <math/Mat3x4f.hpp>

class Pipeline : public RGLAutoCleanupTest {};

TEST_F(Pipeline, Minimal)
{
	setupBoxesAlongAxes(nullptr);

	std::vector<rgl_mat3x4f> rays = loadVec<rgl_mat3x4f>("/home/prybicki/Desktop/rgl/lidar1000-1000-360-180.mat3x4f");
	rgl_node_t use_rays=nullptr, raytrace=nullptr, lidar_pose=nullptr, write=nullptr, compact=nullptr;

	rgl_mat3x4f lidarPose = Mat3x4f::TRS({0, 0, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_pipeline_use_rays_mat3x4f(&use_rays, nullptr, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_pipeline_transform_rays(&lidar_pose, use_rays, &lidarPose));
	EXPECT_RGL_SUCCESS(rgl_pipeline_raytrace(&raytrace, lidar_pose, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_pipeline_compact(&compact, raytrace));
	EXPECT_RGL_SUCCESS(rgl_pipeline_write_pcd_file(&write, compact, "minimal.pcd"));
	EXPECT_RGL_SUCCESS(rgl_pipeline_run(write));
}
