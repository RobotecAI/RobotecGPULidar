#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "utils.hpp"
#include <rgl/api/core.h>
#include <rgl/api/e2e_extensions.h>

using namespace ::testing;

#define VERTICES cube_vertices
#define INDICES cube_indices

class APISurfaceTests : public RGLAutoCleanupTest {};

TEST_F(APISurfaceTests, rgl_configure_logging)
{
	std::filesystem::path logFilePath { std::filesystem::temp_directory_path() / std::filesystem::path("RGL-log.txt") };

	// Setup logging, file should be created
	ASSERT_THAT(logFilePath.c_str(), NotNull());
	ASSERT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_INFO, logFilePath.c_str(), true));
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
	ASSERT_RGL_SUCCESS(rgl_configure_logging(RGL_LOG_LEVEL_OFF, nullptr, false));
}

TEST_F(APISurfaceTests, rgl_get_version_info)
{
	int major, minor, patch;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(nullptr, nullptr, nullptr), "out_major != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  nullptr, nullptr), "out_minor != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  &minor,  nullptr), "out_patch != nullptr");

	ASSERT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_EQ(major, RGL_VERSION_MAJOR);
	EXPECT_EQ(minor, RGL_VERSION_MINOR);
	EXPECT_EQ(patch, RGL_VERSION_PATCH);

}

TEST_F(APISurfaceTests, rgl_mesh_create_destroy)
{
	rgl_mesh_t mesh = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(nullptr, nullptr,   0,                    nullptr, 0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   nullptr,   0,                    nullptr, 0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   VERTICES,  0,                    nullptr, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   VERTICES,  ARRAY_SIZE(VERTICES), nullptr, 0), "indices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_create(&mesh,   VERTICES,  ARRAY_SIZE(VERTICES), INDICES, 0), "index_count > 0");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_mesh_create(&mesh, VERTICES, ARRAY_SIZE(VERTICES), INDICES, ARRAY_SIZE(INDICES)));
	ASSERT_THAT(mesh, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_mesh_destroy(mesh));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_destroy(mesh), "Object does not exist: Mesh");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_destroy(nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_destroy((rgl_mesh_t) 0x1234), "Object does not exist: Mesh 0x1234");

}

TEST_F(APISurfaceTests, rgl_mesh_update_vertices)
{
	rgl_mesh_t mesh = makeCubeMesh();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(nullptr, nullptr,  0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    nullptr,  0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    VERTICES, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    VERTICES, ARRAY_SIZE(VERTICES) + 1), "vertex counts do not match");
	EXPECT_RGL_INVALID_OBJECT(rgl_mesh_update_vertices((rgl_mesh_t) 0x1234, VERTICES, ARRAY_SIZE(VERTICES)), "Object does not exist: Mesh 0x1234");

	// Correct update_vertices
	ASSERT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES)));
}

TEST_F(APISurfaceTests, rgl_entity_create_destroy)
{
	rgl_mesh_t mesh = makeCubeMesh();
	rgl_entity_t entity = nullptr;

	// Invalid args, note: scene can be nullptr here.
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(nullptr, nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(&entity, nullptr, nullptr), "mesh != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_create(&entity, (rgl_scene_t) 0x1234, mesh), "Object does not exist: Scene 0x1234");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_create(&entity, nullptr, (rgl_mesh_t) 0x1234), "Object does not exist: Mesh 0x1234");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	ASSERT_THAT(entity, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_entity_destroy(entity));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_destroy(entity), "Object does not exist: Entity");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_destroy(nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_destroy((rgl_entity_t) 0x1234), "Object does not exist: Entity 0x1234");
}

TEST_F(APISurfaceTests, rgl_entity_set_pose)
{
	rgl_entity_t entity = makeEntity();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(entity,  nullptr), "local_to_world_tf != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_entity_set_pose((rgl_entity_t) 0x1234, &identity), "Object does not exist: Entity 0x1234");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identity));
}

TEST_F(APISurfaceTests, rgl_lidar_create_destroy)
{
	rgl_lidar_t lidar = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_create(nullptr, nullptr,   0), "out_lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_create(&lidar,  nullptr,   0), "ray_transforms != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_create(&lidar,  &identity, 0), "ray_transforms_count > 0");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_lidar_create(&lidar, &identity, 1));
	ASSERT_THAT(lidar, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_lidar_destroy(lidar));

	// Double destroy
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_destroy(lidar), "Lidar");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_destroy(nullptr), "lidar != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_destroy((rgl_lidar_t) 0x1234), "Object does not exist: Lidar 0x1234");
}

TEST_F(APISurfaceTests, rgl_lidar_set_range)
{
	rgl_lidar_t lidar = makeTrivialLidar();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_range(nullptr,  0.0f), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_range(lidar,   -1.0f), "range > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_range(lidar,    0.0f), "range > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_range(lidar,    NAN), "!std::isnan(range)");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_set_range((rgl_lidar_t) 0x1234, 1.0f), "Object does not exist: Lidar 0x1234");

	// Correct set_range
	EXPECT_RGL_SUCCESS(rgl_lidar_set_range(lidar, 1.0f));
}

TEST_F(APISurfaceTests, rgl_lidar_set_pose)
{
	rgl_lidar_t lidar = makeTrivialLidar();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_pose(nullptr, nullptr), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_pose(lidar, nullptr), "local_to_world_tf != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_set_pose((rgl_lidar_t) 0x1234, &identity), "Object does not exist: Lidar 0x1234");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_lidar_set_pose(lidar, &identity));
}

TEST_F(APISurfaceTests, rgl_lidar_raytrace_async)
{
	rgl_lidar_t lidar = makeTrivialLidar();

	// Invalid args raytrace_async
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_raytrace_async(nullptr, nullptr), "lidar != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_raytrace_async((rgl_scene_t) 0x1234, lidar), "Object does not exist: Scene 0x1234");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_raytrace_async(nullptr, (rgl_lidar_t) 0x1234), "Object does not exist: Lidar 0x1234");

	// Correct raytrace_async
	EXPECT_RGL_SUCCESS(rgl_lidar_raytrace_async(nullptr, lidar));
}

TEST_F(APISurfaceTests, rgl_lidar_get_output)
{
	rgl_lidar_t lidar = makeTrivialLidar();
	int hitpointCount = -1;

	// Invalid args get_output_size
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_get_output_size(nullptr, nullptr), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_get_output_size(lidar, nullptr), "out_size != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_get_output_size((rgl_lidar_t) 0x1234, &hitpointCount), "Object does not exist: Lidar 0x1234");

	// Invalid args get_output_data
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_get_output_data(nullptr, RGL_FORMAT_INVALID, nullptr), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_get_output_data(lidar, RGL_FORMAT_INVALID, nullptr), "formatOK");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, nullptr), "out_data != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_get_output_data((rgl_lidar_t) 0x1234, RGL_FORMAT_XYZ, (void*) 0xCAFEBABE), "Object does not exist: Lidar 0x1234");

	// Correct get_output_size
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_size(lidar, &hitpointCount));
	EXPECT_EQ(hitpointCount, 0);

	// Correct get_output_data
	EXPECT_RGL_SUCCESS(rgl_lidar_get_output_data(lidar, RGL_FORMAT_XYZ, (void*) 0xCAFEBABE));
}

TEST_F(APISurfaceTests, rgl_lidar_set_ring_indices)
{
	rgl_lidar_t lidar = makeTrivialLidar();
	int ring_indices[] = { 0 };

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_ring_indices(nullptr, nullptr, 0), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_ring_indices(lidar, nullptr, 0), "ring_ids != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_ring_indices(lidar, ring_indices, 0), "ring_ids_count > 0");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_set_ring_indices((rgl_lidar_t) 0x1234, ring_indices, ARRAY_SIZE(ring_indices)), "Object does not exist: Lidar 0x1234");

	// Correct set_ring_indices
	EXPECT_RGL_SUCCESS(rgl_lidar_set_ring_indices(lidar, ring_indices, ARRAY_SIZE(ring_indices)));
}

TEST_F(APISurfaceTests, rgl_lidar_set_gaussian_noise_params)
{
	rgl_lidar_t lidar = makeTrivialLidar();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(nullptr, (rgl_angular_noise_type_t) -1,   0.0f,  0.0f,  0.0f,  0.0f, 0.0f), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, (rgl_angular_noise_type_t) -1,     0.0f,  0.0f,  0.0f,  0.0f, 0.0f), "noiseTypeOK");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, RGL_ANGULAR_NOISE_TYPE_RAY_BASED, -1.0f,  0.0f,  0.0f,  0.0f, 0.0f), "angular_noise_stddev >= 0.0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, RGL_ANGULAR_NOISE_TYPE_RAY_BASED,  1.0f,  1.0f, -1.0f,  0.0f, 0.0f), "distance_noise_stddev_base >= 0.0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_gaussian_noise_params(lidar, RGL_ANGULAR_NOISE_TYPE_RAY_BASED,  1.0f,  1.0f,  1.0f, -1.0f, 0.0f), "distance_noise_stddev_rise_per_meter >= 0.0");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_set_gaussian_noise_params((rgl_lidar_t) 0x1234, RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f), "Object does not exist: Lidar 0x1234");

	// Correct set_gaussian_noise_params
	EXPECT_RGL_SUCCESS(rgl_lidar_set_gaussian_noise_params(lidar, RGL_ANGULAR_NOISE_TYPE_RAY_BASED, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f));
}

TEST_F(APISurfaceTests, rgl_lidar_set_post_raycast_transform)
{
	rgl_lidar_t lidar = makeTrivialLidar();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_post_raycast_transform(nullptr, nullptr), "lidar != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_lidar_set_post_raycast_transform(lidar, nullptr), "transform != nullptr");
	EXPECT_RGL_INVALID_OBJECT(rgl_lidar_set_post_raycast_transform((rgl_lidar_t) 0x1234, &identity), "Object does not exist: Lidar 0x1234");

	// Correct set_post_raycast_transform
	EXPECT_RGL_SUCCESS(rgl_lidar_set_post_raycast_transform(lidar, &identity));
}
