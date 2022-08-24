#include <rgl/api/experimental.h>

#include <gtest/gtest.h>
#include <utils/testUtils.h>

#include <spdlog/fmt/fmt.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math/Mat3x4f.hpp>

void captureToPCD(rgl_lidar_t lidar)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.height   = 1;
	cloud.is_dense = true;

	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, (int*) &cloud.width));

	std::vector<rgl_vec3f> output;
	output.resize(cloud.width);
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, output.data()));

	for (auto&& v : output) {
		cloud.push_back({v.value[0], v.value[1], v.value[2]});
	}

	pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
}

TEST(PCD_Output, Godzilla)
{
	rgl_lidar_t lidar = loadLidar("/home/prybicki/Desktop/rgl/lidar2048-1024-240-120.mat3x4f");
	rgl_mesh_t mesh = loadMesh("/home/prybicki/Desktop/rgl/godzilla");
	rgl_entity_t entity = makeEntity(mesh);

	rgl_mat3x4f entityPose = (Mat3x4f::rotation(90, 0, 0) * Mat3x4f::translation(0, 150, 150)).toRGL();
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPose));

	captureToPCD(lidar);
}

TEST(PCD_Output, Orientation)
{
	constexpr int BOX_COUNT = 10;
	constexpr float scaleX = 1.0f;
	constexpr float scaleY = 2.0f;
	constexpr float scaleZ = 3.0f;

	rgl_mesh_t cube = makeCubeMesh();
	std::vector<rgl_entity_t> xs, ys, zs;
	for (int i = 0; i < BOX_COUNT; ++i) {
		xs.push_back(makeEntity(cube));
		ys.push_back(makeEntity(cube));
		zs.push_back(makeEntity(cube));

		rgl_mat3x4f xTf = Mat3x4f::TRS({(2 * scaleX + 2) * i, 0, 0}, {45, 0, 0}, {scaleX, 1, 1}).toRGL();
		rgl_mat3x4f yTf = Mat3x4f::TRS({0, (2 * scaleY + 2) * i, 0}, {0, 45, 0}, {1, scaleY, 1}).toRGL();
		rgl_mat3x4f zTf = Mat3x4f::TRS({0, 0, (2 * scaleZ + 2) * i}, {0, 0, 45}, {1, 1, scaleZ}).toRGL();

		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(xs[i], &xTf));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(ys[i], &yTf));
		EXPECT_RGL_SUCCESS(rgl_entity_set_pose(zs[i], &zTf));
	}

	rgl_lidar_t lidar = loadLidar("/tmp/rgl/lidar1000-1000-360-180.mat3x4f");
	rgl_mat3x4f lidarTf = Mat3x4f::translation(10.0f, 10.0f, 10.0f).toRGL();
	EXPECT_RGL_SUCCESS(rgl_lidar_set_pose(lidar, &lidarTf));
	captureToPCD(lidar);


}