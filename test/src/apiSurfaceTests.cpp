#include <Logger.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utils.hpp>
#include <rgl/api/core.h>
#include <scenes.hpp>

#ifdef RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>
#endif


#ifdef RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
#endif

using namespace ::testing;

#define VERTICES cubeVertices
#define INDICES cubeIndices

class APISurfaceTests : public RGLAutoSetUp
{
	std::string getFilename() override { return FILENAME; }
};

TEST_F(APISurfaceTests, rgl_get_version_info)
{
	int major, minor, patch;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(nullptr, nullptr, nullptr), "out_major != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  nullptr, nullptr), "out_minor != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_get_version_info(&major,  &minor,  nullptr), "out_patch != nullptr");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_get_version_info(&major, &minor, &patch));
	EXPECT_EQ(major, RGL_VERSION_MAJOR);
	EXPECT_EQ(minor, RGL_VERSION_MINOR);
	EXPECT_EQ(patch, RGL_VERSION_PATCH);
}

TEST_F(APISurfaceTests, rgl_get_last_error_string)
{
	// Invalid args
	rgl_get_last_error_string(nullptr);
	Logger::getOrCreate().flush();

	// Expected log should be written in the file
	std::filesystem::path logFilePath { std::filesystem::temp_directory_path() / std::filesystem::path(FILENAME).concat(".log") };
	ASSERT_THAT(std::filesystem::exists(logFilePath), IsTrue());
	std::string logFile = readFileStr(logFilePath);
	ASSERT_FALSE(logFile.empty());
	EXPECT_THAT(logFile, HasSubstr("rgl_get_last_error_string"));
	EXPECT_THAT(logFile, HasSubstr("warn"));

	// Valid args
	const char* error_text;
	rgl_get_last_error_string(&error_text);
	EXPECT_THAT(error_text, NotNull());
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
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_mesh_destroy(mesh), "Object does not exist: Mesh");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_destroy(nullptr), "mesh != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_mesh_destroy((rgl_mesh_t) 0x1234), "Object does not exist: Mesh 0x1234");
}

TEST_F(APISurfaceTests, rgl_mesh_update_vertices)
{
	rgl_mesh_t mesh = makeCubeMesh();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(nullptr, nullptr,  0), "mesh != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    nullptr,  0), "vertices != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    VERTICES, 0), "vertex_count > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_mesh_update_vertices(mesh,    VERTICES, ARRAY_SIZE(VERTICES) + 1), "vertex counts do not match");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_mesh_update_vertices((rgl_mesh_t) 0x1234, VERTICES, ARRAY_SIZE(VERTICES)), "Object does not exist: Mesh 0x1234");

	// Correct update_vertices
	EXPECT_RGL_SUCCESS(rgl_mesh_update_vertices(mesh, VERTICES, ARRAY_SIZE(VERTICES)));
}

TEST_F(APISurfaceTests, rgl_entity_create_destroy)
{
	rgl_mesh_t mesh = makeCubeMesh();
	rgl_entity_t entity = nullptr;

	// Invalid args, note: scene can be nullptr here.
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(nullptr, nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_create(&entity, nullptr, nullptr), "mesh != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_entity_create(&entity, (rgl_scene_t) 0x1234, mesh), "Object does not exist: Scene 0x1234");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_entity_create(&entity, nullptr, (rgl_mesh_t) 0x1234), "Object does not exist: Mesh 0x1234");

	// Correct create
	ASSERT_RGL_SUCCESS(rgl_entity_create(&entity, nullptr, mesh));
	ASSERT_THAT(entity, NotNull());

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_entity_destroy(entity));

	// Double destroy
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_entity_destroy(entity), "Object does not exist: Entity");

	// Invalid destroy
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_destroy(nullptr), "entity != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_entity_destroy((rgl_entity_t) 0x1234), "Object does not exist: Entity 0x1234");
}

TEST_F(APISurfaceTests, rgl_entity_set_pose)
{
	rgl_entity_t entity = makeEntity();

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(nullptr, nullptr), "entity != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_entity_set_pose(entity,  nullptr), "transform != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_entity_set_pose((rgl_entity_t) 0x1234, &identity), "Object does not exist: Entity 0x1234");

	// Correct set_pose
	EXPECT_RGL_SUCCESS(rgl_entity_set_pose(entity, &identity));
}

TEST_F(APISurfaceTests, rgl_node_rays_from_mat3x4f)
{
	rgl_node_t node = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(nullptr, nullptr,   0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&node, 	nullptr,   0), "rays != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_from_mat3x4f(&node, 	&identity, 0), "ray_count > 0");

	// Correct node_rays_from_mat3x4f
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&node, &identity, 1));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&node, &identity, 1));
}

TEST_F(APISurfaceTests, rgl_node_rays_set_ring_ids)
{
	rgl_node_t node = nullptr; 
	const int32_t ring_ids [] = {1, 2, 3};
	
	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(nullptr, nullptr,  0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&node, 	nullptr,  0), "ring_ids != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_set_ring_ids(&node, 	ring_ids, 0), "ring_ids_count > 0");

	// Correct set_ring_ids
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&node, ring_ids, 1));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_set_ring_ids(&node, ring_ids, 1));
}

TEST_F(APISurfaceTests, rgl_node_rays_transform)
{
	rgl_node_t node = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(nullptr, nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_rays_transform(&node,	 nullptr), "transform != nullptr");

	// Correct rays_transform
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&node, &identity));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&node, &identity));
}

TEST_F(APISurfaceTests, rgl_node_points_transform)
{
	rgl_node_t node = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(nullptr, nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_transform(&node,   nullptr), "transform != nullptr");

	// Correct points_transform
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&node, &identity));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&node, &identity));
}

TEST_F(APISurfaceTests, rgl_node_raytrace)
{
	rgl_node_t node = nullptr; 

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(nullptr, nullptr, 0.0f), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_raytrace(&node,   nullptr, 0.0f), "range > 0.0f");

	// Correct raytrace
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&node, nullptr, 1.0f));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&node, nullptr, 1.0f));
}

TEST_F(APISurfaceTests, rgl_node_points_format)
{
	rgl_node_t node = nullptr;
	rgl_field_t fields;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(nullptr, nullptr, 0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&node,   nullptr, 0), "fields != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_format(&node, 	&fields, 0), "field_count > 0");

	// Correct points_format
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&node, &fields, 1));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&node, &fields, 1));
}

TEST_F(APISurfaceTests, rgl_node_points_yield)
{
	rgl_node_t node = nullptr;
	rgl_field_t fields;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(nullptr, nullptr, 0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&node,   nullptr, 0), "fields != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_yield(&node,   &fields, 0), "field_count > 0");

	// Correct points_yield
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&node, &fields, 1));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_yield(&node, &fields, 1));
}

TEST_F(APISurfaceTests, rgl_node_points_compact)
{
	rgl_node_t node = nullptr;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_compact(nullptr), "node != nullptr");

	// Correct points_compact
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&node));
	ASSERT_THAT(node, NotNull());

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&node));
}

TEST_F(APISurfaceTests, rgl_node_points_spatial_merge)
{
	rgl_node_t node = nullptr;
	std::vector<rgl_field_t> sMergeFields = {RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_PADDING_32};

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(nullptr, nullptr, 0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_spatial_merge(&node,   nullptr, 0), "fields != nullptr");

	// Correct points_spatial_merge
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&node, sMergeFields.data(), sMergeFields.size()));

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&node, sMergeFields.data(), sMergeFields.size()));
}

TEST_F(APISurfaceTests, rgl_node_points_temporal_merge)
{
	rgl_node_t node = nullptr;
	std::vector<rgl_field_t> tMergeFields = {RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_PADDING_32};

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(nullptr, nullptr, 0), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_temporal_merge(&node,   nullptr, 0), "fields != nullptr");

	// Correct points_temporal_merge
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&node, tMergeFields.data(), tMergeFields.size()));

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&node, tMergeFields.data(), tMergeFields.size()));
}

TEST_F(APISurfaceTests, rgl_node_gaussian_noise_angular_ray)
{
	rgl_node_t node = nullptr;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_angular_ray(nullptr, 0.0f, 0.0f, RGL_AXIS_X), "node != nullptr");

	// Correct gaussian_noise_angular_ray
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&node, 0.0f, 0.0f, RGL_AXIS_X));

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_angular_ray(&node, 2.0f, 2.0f, RGL_AXIS_X));
}

TEST_F(APISurfaceTests, rgl_node_gaussian_noise_distance)
{
	rgl_node_t node = nullptr;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_gaussian_noise_distance(nullptr, 0.0f, 0.0f, 0.0f), "node != nullptr");

	// Correct gaussian_noise_distance
	EXPECT_RGL_SUCCESS(rgl_node_gaussian_noise_distance(&node, 0.1f, 0.1f, 0.01f));
}

TEST_F(APISurfaceTests, rgl_graph_run)
{
	rgl_node_t node = nullptr, raytrace = nullptr;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_run(nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_run(node), "node != nullptr");

	// Creating graph with two nodes
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&node, &identity, 1));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(node, raytrace));

	// Correct graph_run
	EXPECT_RGL_SUCCESS(rgl_graph_run(node));
}

TEST_F(APISurfaceTests, rgl_graph_destroy)
{
	rgl_node_t ray_node = nullptr, raytrace_node = nullptr;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_destroy(nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_destroy(ray_node), "node != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_destroy((rgl_node_t) 0x1234), "Object does not exist: Node 0x1234");

	// Creating nodes
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&ray_node, &identity, 1));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace_node, nullptr, 1000));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(ray_node, raytrace_node));

	// Correct destroy
	ASSERT_RGL_SUCCESS(rgl_graph_destroy(ray_node));

	// Double destroy
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_destroy(raytrace_node), "Object does not exist: Node");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_destroy(ray_node), "Object does not exist: Node");
}

TEST_F(APISurfaceTests, rgl_graph_get_result_size)
{
	rgl_node_t ray_node = nullptr, raytrace_node = nullptr;
	int32_t hitpoint_count, point_size;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_size(nullptr, RGL_FIELD_XYZ_F32, &hitpoint_count, &point_size), "node != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_get_result_size((rgl_node_t) 0x1234, RGL_FIELD_XYZ_F32, &hitpoint_count, &point_size), "Object does not exist: Node 0x1234");

	// Creating graph with two nodes
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&ray_node, &identity, 1));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace_node, nullptr, 1000));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(ray_node, raytrace_node));

	// Correct get_result_size (without prior running the graph)
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytrace_node, RGL_FIELD_XYZ_F32, nullptr, nullptr));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytrace_node, RGL_FIELD_XYZ_F32, &hitpoint_count, &point_size));

	// Correct get_result_size (with prior running the graph)
	ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace_node));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytrace_node, RGL_FIELD_XYZ_F32, nullptr, nullptr));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytrace_node, RGL_FIELD_XYZ_F32, &hitpoint_count, &point_size));
}

TEST_F(APISurfaceTests, rgl_graph_get_result_data)
{
	rgl_node_t ray_node = nullptr, raytrace_node = nullptr;
	int32_t hitpoint_count, point_size;
	rgl_vec3f results[1];

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(nullptr, 	RGL_FIELD_XYZ_F32, nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(ray_node, RGL_FIELD_XYZ_F32, nullptr), "node != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_get_result_data((rgl_node_t) 0x1234, RGL_FIELD_XYZ_F32, &results), "Object does not exist: Node 0x1234");

	// Invalid arg (data)
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&ray_node, &identity, 1));
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(ray_node, RGL_FIELD_XYZ_F32, nullptr), "data != nullptr");

	// Creating graph with two nodes
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytrace_node, nullptr, 1000));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(ray_node, raytrace_node));

	// Incorrect type of node
	EXPECT_RGL_NODE_TYPE_MISMATCH(rgl_graph_get_result_data(ray_node, RGL_FIELD_XYZ_F32, &results), "expected IPointsNode, got FromMat3x4fRaysNode");

	// Correct get_result_data (without prior running the graph)
	// EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_get_result_data(raytrace_node, RGL_FIELD_XYZ_F32, &results), "");

	// Correct get_result_data (with prior running the graph)
	ASSERT_RGL_SUCCESS(rgl_graph_run(raytrace_node));
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace_node, RGL_FIELD_XYZ_F32, &results));
}

TEST_F(APISurfaceTests, rgl_graph_node_add_child)
{
	rgl_node_t parent = nullptr, child = nullptr;

	// Invalid arg (parent)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_add_child(nullptr, nullptr), "parent != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_add_child(parent,  nullptr), "parent != nullptr");

	// Creating rays node - parent
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&parent, &identity, 1));

	// Invalid arg (child)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_add_child(parent,  nullptr), "child != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_add_child(parent,  child), 	"child != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_node_add_child(parent, (rgl_node_t) 0x1234), "Object does not exist: Node 0x1234");

	// Creating raytrace node - child
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&child, nullptr, 1000));

	// Invalid arg (parent does not exist)
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_node_add_child((rgl_node_t) 0x1234, child), "Object does not exist: Node 0x1234");

	// Correct add_child
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(parent, child));
}

TEST_F(APISurfaceTests, rgl_graph_node_remove_child)
{
	rgl_node_t parent = nullptr, child = nullptr;

	// Invalid arg (parent)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_remove_child(nullptr, nullptr), "parent != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_remove_child(parent,  nullptr), "parent != nullptr");

	// Creating rays node
	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&parent, &identity, 1));

	// Invalid arg (child)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_remove_child(parent,  nullptr),  "child != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_graph_node_remove_child(parent,  child), 	"child != nullptr");
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_node_remove_child(parent, (rgl_node_t) 0x1234), "Object does not exist: Node 0x1234");

	// Creating raytrace node
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&child, nullptr, 1000));

	// Invalid arg (parent does not exist)
	EXPECT_RGL_OBJECT_NOT_EXISTS(rgl_graph_node_remove_child((rgl_node_t) 0x1234, child), "Object does not exist: Node 0x1234");

	// Invalid args - nodes are not connected
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_node_remove_child(parent, child), "RaytraceNode from FromMat3x4fRaysNode");

	// Connecting nodes
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(parent, child));

	// Invalid args - incorrect order
	EXPECT_RGL_INVALID_PIPELINE(rgl_graph_node_remove_child(child, parent), "FromMat3x4fRaysNode from RaytraceNode");

	// Valid args
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(parent, child));
}

#ifdef RGL_BUILD_PCL_EXTENSION
TEST_F(APISurfaceTests, rgl_node_points_downsample)
{
	rgl_node_t node = nullptr;

	// Invalid arg (node)
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_downsample(nullptr, 0.0f, 0.0f, 0.0f), "node != nullptr");

	// Correct points_downsample
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&node, 1.0f, 1.0f, 1.0f));

	// If (*node) != nullptr
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&node, 2.0f, 2.0f, 2.0f));
}

TEST_F(APISurfaceTests, rgl_node_points_visualize)
{
	rgl_node_t node = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_visualize(nullptr, nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_visualize(&node,   nullptr), "window_name != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_visualize(&node,   "window", 0), "window_width > 0");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_visualize(&node,   "window", 1280, 0), "window_height > 0");

	// Skipping valid args - avoiding the pop-up window
}
#endif

#ifdef RGL_BUILD_ROS2_EXTENSION
TEST_F(APISurfaceTests, rgl_node_points_ros2_publish)
{
	rgl_node_t node = nullptr;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish(nullptr, nullptr, 		nullptr), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish(&node,   nullptr, 		nullptr), "topic_name != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish(&node,   "topic_name", nullptr), "frame_id != nullptr");

	// Correct points_ros2_publish
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish(&node, "topic_name", "frame_id"));
}

TEST_F(APISurfaceTests, rgl_node_points_ros2_publish_with_qos)
{
	rgl_node_t node = nullptr;
	rgl_qos_policy_reliability_t qos_r = QOS_POLICY_RELIABILITY_BEST_EFFORT;
	rgl_qos_policy_durability_t qos_d = QOS_POLICY_DURABILITY_VOLATILE;
	rgl_qos_policy_history_t qos_h = QOS_POLICY_HISTORY_KEEP_LAST;

	// Invalid args
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish_with_qos(nullptr, nullptr, 	 nullptr, 	 qos_r, qos_d, qos_h, -1), "node != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish_with_qos(&node,   nullptr, 	 nullptr, 	 qos_r, qos_d, qos_h, -1), "topic_name != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish_with_qos(&node,   "topic_name", nullptr, 	 qos_r, qos_d, qos_h, -1), "frame_id != nullptr");
	EXPECT_RGL_INVALID_ARGUMENT(rgl_node_points_ros2_publish_with_qos(&node,   "topic_name", "frame_id", qos_r, qos_d, qos_h, -1), "qos_history_depth >= 0");

	// Correct points_ros2_publish_with_qos
	EXPECT_RGL_SUCCESS(rgl_node_points_ros2_publish_with_qos(&node, "topic_name", "frame_id", qos_r, qos_d, qos_h, 0));
}
#endif