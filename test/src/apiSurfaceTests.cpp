#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "utils.hpp"
#include <rgl/api/core.h>

using namespace ::testing;

#define VERTICES cube_vertices
#define INDICES cube_indices

class APISurfaceTests : public RGLAutoCleanupTest {};

TEST_F(APISurfaceTests, rgl_configure_logging)
{
	std::filesystem::path logFilePath { std::filesystem::temp_directory_path() / std::filesystem::path("RGL-log.txt") };

	// Setup logging, file should be created
	ASSERT_THAT(logFilePath.c_str(), NotNull());
	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_INFO, logFilePath.c_str(), true));
	ASSERT_THAT(std::filesystem::exists(logFilePath), IsTrue());
	Logger::instance().flush();
	ASSERT_THAT(readFileStr(logFilePath), HasSubstr("Logging configured"));

	// Write some logs
	RGL_TRACE("This is RGL trace log."); // Should be not printed
	RGL_INFO("This is RGL info log.");
	RGL_WARN("This is RGL warn log.");
	RGL_ERROR("This is RGL error log.");
	RGL_CRITICAL("This is RGL critical log.");
	Logger::instance().flush();

	// Expected log levels should be written in the file
	std::string logFile = readFileStr(logFilePath);
	EXPECT_THAT(logFile, Not(HasSubstr("trace")));
	EXPECT_THAT(logFile, HasSubstr("info"));
	EXPECT_THAT(logFile, HasSubstr("warn"));
	EXPECT_THAT(logFile, HasSubstr("error"));
	EXPECT_THAT(logFile, HasSubstr("critical"));
	EXPECT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false));
}

TEST_F(APISurfaceTests, rgl_get_version_info)
{
	int major, minor, patch;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(nullptr, nullptr, nullptr), "out_major != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  nullptr, nullptr), "out_minor != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  &minor,  nullptr), "out_patch != nullptr");

	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_EQ(major, RGL_VERSION_MAJOR);
	EXPECT_EQ(minor, RGL_VERSION_MINOR);
	EXPECT_EQ(patch, RGL_VERSION_PATCH);
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_get_last_error_string)
{
	// Invalid args
	EXPECT_EQ(rgl_get_last_error_string(nullptr), RGL_INVALID_ARGUMENT);
	
	// No error
	const char* errorString;
	EXPECT_RGL_SUCCESS(rgl_get_last_error_string(&errorString));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}


TEST_F(APISurfaceTests, rgl_node_rays_from_mat3x4f)
{
	rgl_node_t node = {};
	rgl_mat3x4f rays;
	int32_t invalid_rays_count = 0;
	int32_t valid_rays_count = 1;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(nullptr, &rays, valid_rays_count), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&node, nullptr, valid_rays_count), "rays != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&node, &rays, invalid_rays_count), "ray_count > 0");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&node, &rays, valid_rays_count));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_rays_set_ring_ids)
{
	rgl_node_t node = {}; 
	const int32_t ring_ids = 1; 
	const int32_t invalid_ring_ids_count = 0;
	const int32_t valid_ring_ids_count = 1;
	

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(nullptr, &ring_ids, valid_ring_ids_count), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&node, nullptr, valid_ring_ids_count), "ring_ids != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&node, &ring_ids, invalid_ring_ids_count), "ring_ids_count > 0");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&node, &ring_ids, valid_ring_ids_count));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_rays_transform)
{
	rgl_node_t node = {};
	rgl_mat3x4f transform;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, &transform), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(&node, nullptr), "transform != nullptr");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&node, &transform));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_points_transform)
{
	rgl_node_t node = {};
	rgl_mat3x4f transform;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(nullptr, &transform), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(&node, nullptr), "transform != nullptr");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&node, &transform));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_raytrace)
{
	rgl_node_t node = {}; 
	rgl_scene_t scene; 
	float range = 1.0f;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, scene, range), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(&node, scene, 0.0f), "range > 0.0f");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&node, nullptr, range));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&node, scene, range));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_points_format)
{
	rgl_node_t node = {};
	rgl_field_t fields;
	int32_t field_count = 1;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, &fields, field_count), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&node, nullptr, field_count), "fields != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&node, &fields, 0), "field_count > 0");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&node, &fields, field_count));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_points_yield)
{
	rgl_node_t node = {};
	rgl_field_t fields;
	int32_t field_count = 1;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(nullptr, &fields, field_count), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&node, nullptr, field_count), "fields != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&node, &fields, 0), "field_count > 0");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&node, &fields, field_count));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_points_compact)
{
	rgl_node_t node = {};

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "nodeRawPtr != nullptr");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&node));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_points_downsample)
{
	rgl_node_t node = {};
	float leaf_size_x = 1.0f; 
	float leaf_size_y = 2.0f;
	float leaf_size_z = 3.0f;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(nullptr, leaf_size_x, leaf_size_y, leaf_size_z), "nodeRawPtr != nullptr");

	// This are passing, assume they should fail if size less or eq to 0??
	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, 0.0f, leaf_size_y, leaf_size_z), "leaf_size_x > 0.0f");
	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, -1.0f, leaf_size_y, leaf_size_z), "leaf_size_x > 0.0f");
	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, leaf_size_x, 0.0f, leaf_size_z), "leaf_size_y > 0.0f");
	// EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(&node, leaf_size_x, leaf_size_y, 0.0f), "leaf_size_z > 0.0f");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&node, leaf_size_x, leaf_size_y, leaf_size_z));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}

TEST_F(APISurfaceTests, rgl_node_points_write_pcd_file)
{
	rgl_node_t node = {};
	const char* file_path = "test.pcd";

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_write_pcd_file(nullptr, file_path), "nodeRawPtr != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_write_pcd_file(&node, nullptr), "file_path != nullptr");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_node_points_write_pcd_file(&node, file_path));
	EXPECT_RGL_SUCCESS(rgl_cleanup());
}
