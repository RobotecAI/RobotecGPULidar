#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>

#include <math/Mat3x4f.hpp>

class Pipeline : public RGLAutoCleanupTest {};

TEST_F(Pipeline, Minimal)
{
	setupBoxesAlongAxes(nullptr);

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, shear=nullptr, compact=nullptr, downsample=nullptr, write=nullptr;

	std::vector<rgl_mat3x4f> rays = loadVec<rgl_mat3x4f>("/home/prybicki/Desktop/rgl/lidar1000-1000-360-180.mat3x4f");
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({5, 5, 5}, {45, 45, 45}).toRGL();
	rgl_mat3x4f shearTf = Mat3x4f::shear({0,0}, {-1, -1}, {0, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_pipeline_use_rays_mat3x4f(&useRays, nullptr, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_pipeline_transform_rays(&lidarPose, useRays, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_pipeline_raytrace(&raytrace, lidarPose, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_pipeline_compact(&compact, raytrace));
	EXPECT_RGL_SUCCESS(rgl_pipeline_transform_points(&shear, compact, &shearTf));
	EXPECT_RGL_SUCCESS(rgl_pipeline_downsample(&downsample, shear, 0.1f, 0.1f, 0.1f));
	EXPECT_RGL_SUCCESS(rgl_pipeline_write_pcd_file(&write, shear, "minimal.pcd"));
	EXPECT_RGL_SUCCESS(rgl_pipeline_run(write));
}
