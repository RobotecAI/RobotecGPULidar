#include <rgl/api/experimental.h>

#include <gtest/gtest.h>
#include <utils/testUtils.h>

#include <spdlog/fmt/fmt.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math/Mat3x4f.hpp>

rgl_mat3x4f getTfX(float x, float y, float z, float angleX)
{
	return {
	.value = {
	{ 1, 0, 0, x },
	{ 0, std::cos(angleX), -std::sin(angleX), y },
	{ 0, std::sin(angleX), std::cos(angleX), z },
	}
	};
}

TEST(PCD_Output, Godzilla)
{
	std::vector<rgl_vec3f> godzillaVs = readFileVec<rgl_vec3f>("/tmp/rgl/godzilla.vertices");
	std::vector<rgl_vec3i> godzillaIs = readFileVec<rgl_vec3i>("/tmp/rgl/godzilla.indices");
	std::vector<rgl_mat3x4f> rays = readFileVec<rgl_mat3x4f>("/tmp/rgl/lidar2048-1024-240-120.mat3x4f");
	rgl_mesh_t mesh = nullptr;
	rgl_entity_t entity = nullptr;
	rgl_lidar_t lidar = nullptr;

	rgl_mat3x4f entityPose = getTfX(-50, 50, 200, 90);
	//T Y-100 Z 100
	//R X-90 Z=180

	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, godzillaVs.data(), godzillaVs.size(), godzillaIs.data(), godzillaIs.size()));
	EXPECT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPose));
	EXPECT_RGL_SUCCESS(rgl_lidar_create(&lidar, rays.data(), rays.size()));

	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.height   = 1;
	cloud.is_dense = true;

	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, (int*) &cloud.width));

	std::vector<rgl_vec3f> output;
	output.resize(cloud.width);
	fmt::print("{} / {}\n", output.size(), sizeof(rgl_vec3f) * output.size());
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, output.data()));

	for (auto&& v : output) {
		cloud.push_back({v.value[0], v.value[1], v.value[2]});
	}

	pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

	// for (auto&& v : godzillaVs) {
	// 	fmt::print("{} {} {}\n", v.value[0], v.value[1], v.value[2]);
	// }
	//
	// for (auto&& v : godzillaIs) {
	// 	fmt::print("{} {} {}\n", v.value[0], v.value[1], v.value[2]);
	// }
}