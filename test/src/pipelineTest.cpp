#include <gtest/gtest.h>
#include <utils/testUtils.h>

#include <math/Mat3x4f.hpp>

class Pipeline : RGLAutoCleanupTest {};

TEST(Pipeline, Minimal)
{
	rgl_configure_logging(RGL_LOG_LEVEL_TRACE, nullptr, true);

	rgl_entity_t entity = makeEntity();
	rgl_mat3x4f entityPose = Mat3x4f::translation(0, 0, 5).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPose));
	std::vector<rgl_mat3x4f> rays = loadVec<rgl_mat3x4f>("/home/prybicki/Desktop/rgl/lidar1000-1000-360-180.mat3x4f");

	rgl_node_t use_rays=nullptr, raytrace=nullptr, write=nullptr;
	EXPECT_RGL_SUCCESS(rgl_pipeline_use_rays_mat3x4f(&use_rays, nullptr, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_pipeline_raytrace(&raytrace, use_rays, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_pipeline_write_pcd_file(&write, raytrace, "output.pcd"));
	EXPECT_RGL_SUCCESS(rgl_pipeline_run(write));
}
