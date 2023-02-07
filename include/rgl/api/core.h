// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define NO_MANGLING extern "C"
#else // NOT __cplusplus
#define NO_MANGLING
#endif

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
  #define RGL_VISIBLE __attribute__ ((dllexport))
 #else
  #define RGL_VISIBLE __declspec(dllexport)
 #endif // __GNUC__
#else
#define RGL_VISIBLE __attribute__ ((visibility("default")))
#if __GNUC__ >= 4
#define RGL_VISIBLE __attribute__ ((visibility("default")))
#else
#define RGL_VISIBLE
#endif
#endif // _WIN32 || __CYGWIN__

#define RGL_API NO_MANGLING RGL_VISIBLE

#define RGL_VERSION_MAJOR 0
#define RGL_VERSION_MINOR 12
#define RGL_VERSION_PATCH 0

/**
 * Three consecutive 32-bit floats.
 */
typedef struct
{
	float value[3];
} rgl_vec3f;

#ifndef __cplusplus
static_assert(sizeof(rgl_vec3f) == 3 * sizeof(float));
#endif

/**
 * Three consecutive 32-bit signed integers.
 */
typedef struct
{
	int32_t value[3];
} rgl_vec3i;

/**
 * Row-major matrix with 3 rows and 4 columns of 32-bit floats.
 */
typedef struct
{
	float value[3][4];
} rgl_mat3x4f;

/**
 * Represents on-GPU mesh that can be referenced by entities on the scene.
 * Each mesh can be referenced by any number of entities on different scenes.
 */
typedef struct Mesh *rgl_mesh_t;

/**
 * Opaque handle representing an object visible to lidars.
 * An entity is always bound to exactly one scene.
 */
typedef struct Entity *rgl_entity_t;

/**
 * TODO(prybicki)
 */
typedef struct Node *rgl_node_t;

/**
 * Opaque handle representing a scene - a collection of entities.
 * Using scene is optional. NULL can be passed to use an implicit default scene.
 */
typedef struct Scene *rgl_scene_t;

/**
 * Status (error) codes returned by all RGL API functions.
 * Unrecoverable errors require reloading the library (restarting the application).
 */
typedef enum
{
	/**
	 * Operation successful.
	 * This is a recoverable error :)
	 */
	RGL_SUCCESS = 0,

	/**
	 * One of the arguments is invalid (e.g. null pointer) or a number in invalid range.
	 * Get the error string for more details.
	 * This is a recoverable error.
	 */
	RGL_INVALID_ARGUMENT,

	/**
	 * RGL internal state has been corrupted by a previous unrecoverable error.
	 * Application must be restarted.
	 */
	RGL_INVALID_STATE,

	/**
	 * Indicates that a logging operation (e.g. configuration) was not successful.
	 * This is an unrecoverable error.
	 */
	RGL_LOGGING_ERROR,

	/**
	 * Indicates that provided API object handle is not known by RGL.
	 * This can be caused by using previously destroyed API object, e.g.
	 * by previous call to rgl_*_destroy(...) or rgl_cleanup()
	 * This is a recoverable error.
	 */
	RGL_INVALID_API_OBJECT,

	/**
	* Indicates that a given file could not be opened.
	* This is a recoverable error.
	*/
	RGL_INVALID_FILE_PATH,

	/**
	* Indicates that a tape operation was not successful.
	* This is a recoverable error.
	*/
	RGL_TAPE_ERROR,

	/**
	* Indicates that a ROS2 native library throws exception.
	* This is a recoverable error.
	*/
	RGL_ROS2_ERROR,

	/**
	 * Indicates an error in the pipeline, such as adjacency of incompatible nodes.
	 * This is a recoverable error.
	 */
	RGL_INVALID_PIPELINE,

	/**
	 * Indicates a failure during (lazy) initialization.
	 * This is an unrecoverable error.
	 */
	RGL_INITIALIZATION_ERROR,

	/**
	 * Requested functionality has been not yet implemented.
	 * This is a recoverable error.
	 */
	RGL_NOT_IMPLEMENTED = 404,

	/**
	 * An unhandled internal error has occurred.
	 * If you see this error, please file a bug report.
	 * This is an unrecoverable error.
	 */
	RGL_INTERNAL_EXCEPTION = 500,
} rgl_status_t;

/**
 * Available logging verbosity levels.
 */
typedef enum : int
{
	RGL_LOG_LEVEL_ALL = 0,
	RGL_LOG_LEVEL_TRACE = 0,
	RGL_LOG_LEVEL_DEBUG = 1,
	RGL_LOG_LEVEL_INFO = 2,
	RGL_LOG_LEVEL_WARN = 3,
	RGL_LOG_LEVEL_ERROR = 4,
	RGL_LOG_LEVEL_CRITICAL = 5,
	RGL_LOG_LEVEL_OFF = 6,
	RGL_LOG_LEVEL_COUNT = 7
} rgl_log_level_t;

/**
 * Available point attributes, used to specify layout of the binary data.
 */
typedef enum
{
	RGL_FIELD_XYZ_F32 = 1,
	RGL_FIELD_INTENSITY_F32,
	RGL_FIELD_IS_HIT_I32,
	RGL_FIELD_RAY_IDX_U32,
	RGL_FIELD_POINT_IDX_U32,
	RGL_FIELD_DISTANCE_F32,
	RGL_FIELD_AZIMUTH_F32,
	RGL_FIELD_RING_ID_U16,
	RGL_FIELD_RETURN_TYPE_U8,
	RGL_FIELD_TIME_STAMP_F64,
	// Dummy fields
	RGL_FIELD_PADDING_8 = 1024,
	RGL_FIELD_PADDING_16,
	RGL_FIELD_PADDING_32,
	// Dynamic fields
	RGL_FIELD_DYNAMIC_FORMAT = 13842,
} rgl_field_t;

/******************************** GENERAL ********************************/

/**
 * Returns data describing semantic version as described in https://semver.org/
 * Version string can be obtained by formatting "{out_major}.{out_minor}.{out_patch}".
 * Hash is provided mostly for debugging and issue reporting.
 * @param out_major Address to store major version number
 * @param out_minor Address to store minor version number
 * @param out_patch Address to store patch version number
 */
RGL_API rgl_status_t
rgl_get_version_info(int32_t *out_major, int32_t *out_minor, int32_t *out_patch);

/**
 * Optionally (re)configures internal logging. This feature may be useful for debugging / issue reporting.
 * By default (i.e. not calling `rgl_configure_logging`) is equivalent to the following call:
 * `rgl_configure_logging(RGL_LOG_LEVEL_INFO, nullptr, true)`
 * @param log_level Controls severity of emitted logs: trace=0, debug=1, info=2, warn=3, error=4, critical=5, off=6
 * @param log_file_path Path to the file where logs will be saved.
 * The file will be created or truncated on each configuration.
 * Pass nullptr to disable logging to file
 * @param use_stdout If true, logs will be outputted to stdout.
 */
RGL_API rgl_status_t
rgl_configure_logging(rgl_log_level_t log_level, const char* log_file_path, bool use_stdout);

/**
 * Returns a pointer to a string explaining last error. This function always succeeds.
 * Returned pointer is valid only until next RGL API call.
 * @param out_error Address to store pointer to string explaining the cause of the given error.
 */
RGL_API void
rgl_get_last_error_string(const char **out_error_string);

/**
 * Removes all user-created API objects: meshes, entities, scenes, lidars, etc.
 * Effectively brings the library to the state as-if it was not yet used.
 * All API handles are invalidated.
 */
RGL_API rgl_status_t
rgl_cleanup(void);


/******************************** MESH ********************************/

/**
 * Creates mesh from vertex and index arrays. CW/CCW order does not matter.
 * Provided arrays are copied to the GPU before this function returns.
 * @param out_mesh Address to store the resulting mesh handle
 * @param vertices An array of rgl_vec3f or binary-compatible data representing mesh vertices
 * @param vertex_count Number of elements in the vertices array
 * @param indices An array of rgl_vec3i or binary-compatible data representing mesh indices
 * @param index_count Number of elements in the indices array
 */
RGL_API rgl_status_t
rgl_mesh_create(rgl_mesh_t *out_mesh,
                const rgl_vec3f *vertices,
                int32_t vertex_count,
                const rgl_vec3i *indices,
                int32_t index_count);

/**
 * Informs that the given mesh will be no longer used.
 * The mesh will be destroyed after all referring entities are destroyed.
 * @param mesh Mesh to be marked as no longer needed
 */
RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh);

/**
 * Updates mesh vertex data. The number of vertices must not change.
 * This function is intended to update animated meshes.
 * @param mesh Mesh to modify
 * @param vertices An array of rgl_vec3f or binary-compatible data representing mesh vertices
 * @param vertex_count Number of elements in the vertices array. Must be equal to the original vertex count!
 */
RGL_API rgl_status_t
rgl_mesh_update_vertices(rgl_mesh_t mesh,
                         const rgl_vec3f *vertices,
                         int32_t vertex_count);


/******************************** ENTITY ********************************/

/**
 * Creates an entity and adds it to the given scene.
 * Entity is a lightweight object which pairs a reference to a mesh with a 3D affine transform.
 * @param out_entity Handle to the created entity.
 * @param scene Scene where entity will be added. Pass NULL to use the default scene.
 * @param mesh Handle to the mesh which will represent the entity on the scene.
 */
RGL_API rgl_status_t
rgl_entity_create(rgl_entity_t *out_entity, rgl_scene_t scene, rgl_mesh_t mesh);

/**
 * Removes an entity from the scene and releases its resources (memory).
 * This operation does not affect the mesh used by the entity, since it can be shared among other entities.
 * @param entity Entity to remove
 */
RGL_API rgl_status_t
rgl_entity_destroy(rgl_entity_t entity);

/**
 * Changes transform (position, rotation, scaling) of the given entity.
 * @param entity Entity to modify
 * @param transform Pointer to rgl_mat3x4f (or binary-compatible data) representing desired (entity -> world) coordinate system transform.
 */
RGL_API rgl_status_t
rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f *local_to_world_tf);

/******************************** SCENE ********************************/

/**
 * Sets time to the given scene.
 * Time indicates a specific point in time, relative to a clock's 0 point.
 * Timestamp is used to fill field RGL_FIELD_TIME_STAMP_F64 or for ROS2 publishing.
 * @param scene Scene where time will be set. Pass NULL to use the default scene.
 * @param nanoseconds Timestamp in nanoseconds.
 */
RGL_API rgl_status_t
rgl_scene_set_time(rgl_scene_t scene, uint64_t nanoseconds);

/******************************** NODES ********************************/

/**
 * Creates or modifies UseRaysMat3x4fNode.
 * The node provides initial rays for its children nodes.
 * Initial rays are usually provided in device-local coordinate frame, i.e. close to (0, 0, 0).
 * Input: none
 * Output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param rays Pointer to 3x4 affine matrices describing rays poses.
 * @param ray_count Size of the `rays` array
 */
RGL_API rgl_status_t
rgl_node_rays_from_mat3x4f(rgl_node_t* node, const rgl_mat3x4f* rays, int32_t ray_count);

/**
 * Creates or modifies UseRingIdsNode.
 * The node assigns ring ids for existing rays.
 * Input: rays
 * Output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param ring_ids Pointer to ring ids.
 * @param ray_count Size of the `ring_ids` array.
 */
RGL_API rgl_status_t
rgl_node_rays_set_ring_ids(rgl_node_t* node, const int32_t* ring_ids, int32_t ring_ids_count);

/**
 * Creates or modifies TransformRaysNode.
 * Effectively, the node performs the following operation for all rays: `outputRay[i] = (*transform) * inputRay[i]`
 * This function can be used to account for the pose of the device.
 * Graph input: rays
 * Graph output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param transform Pointer to a single 3x4 affine matrix describing the transformation to be applied.
 */
RGL_API rgl_status_t
rgl_node_rays_transform(rgl_node_t* node, const rgl_mat3x4f* transform);

// Applies affine transformation, e.g. to change the coordinate frame.
/**
 * Creates or modifies TransformPointsNode.
 * The node applies affine transformation to all points.
 * It can be used to e.g. change coordinate frame after raytracing.
 * Note: affects only RGL_FIELD_XYZ_F32. Other fields are not modified.
 * Graph input: point cloud
 * Graph output: point cloud (transformed)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param transform Pointer to a single 3x4 affine matrix describing the transformation to be applied.
 */
RGL_API rgl_status_t
rgl_node_points_transform(rgl_node_t* node, const rgl_mat3x4f* transform);

/**
 * Creates or modifies RaytraceNode.
 * The node performs GPU-accelerated raytracing on the given scene.
 * Fields to be computed will be automatically determined based on connected FormatNodes and YieldPointsNodes
 * Graph input: rays
 * Graph output: point cloud (sparse)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param scene Handle to a scene to perform raytracing on. Pass null to use the default scene
 * @param range Maximum distance to travel for every ray
 */
RGL_API rgl_status_t
rgl_node_raytrace(rgl_node_t* node, rgl_scene_t scene, float range);

/**
 * Creates or modifies FormatNode.
 * The node converts internal representation into a binary format defined by `fields` array.
 * Note: It is a user's responsibility to ensure proper data structure alignment. See (https://en.wikipedia.org/wiki/Data_structure_alignment).
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param fields Subsequent fields to be present in the binary output
 * @param field_count Number of elements in the `fields` array
 */
RGL_API rgl_status_t
rgl_node_points_format(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies YieldPointsNode.
 * The node is a marker what fields are expected by the user.
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param fields Subsequent fields expected to be available
 * @param field_count Number of elements in the `fields` array
 */
RGL_API rgl_status_t
rgl_node_points_yield(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies CompactNode.
 * The node removes non-hit points. In other words, it converts a point cloud into a dense one.
 * Graph input: point cloud
 * Graph output: point cloud (compacted)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 */
RGL_API rgl_status_t
rgl_node_points_compact(rgl_node_t* node);

/**
 * Creates or modifies DownSampleNode.
 * The node uses VoxelGrid down-sampling filter from PCL library to reduce the number of points.
 * Graph input: point cloud
 * Graph output: point cloud (downsampled)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param leaf_size_* Dimensions of the leaf voxel passed to VoxelGrid filter.
 */
RGL_API rgl_status_t
rgl_node_points_downsample(rgl_node_t* node, float leaf_size_x, float leaf_size_y, float leaf_size_z);

/**
 * Creates or modifies WritePCDFileNode.
 * The node accumulates (merges) point clouds on each run. On destruction, it saves it to the given file.
 * Graph input: point cloud
 * Graph output: none
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param file_path Path to the output pcd file.
 */
RGL_API rgl_status_t
rgl_node_points_write_pcd_file(rgl_node_t* node, const char* file_path);

/******************************** GRAPH ********************************/

/**
 * Starts execution of the RGL graph containing provided node.
 * This function is asynchronous.
 * @param node Any node from the graph to execute
 */
RGL_API rgl_status_t
rgl_graph_run(rgl_node_t node);

/**
 * Destroys RGL graph (all connected nodes) containing provided node.
 * @param node Any node from the graph to destroy
 */
RGL_API rgl_status_t
rgl_graph_destroy(rgl_node_t node);

/**
 * Obtains the result information of any node in the graph.
 * The function will fill output parameters that are not null.
 * I.e. The count of the output elements can be queried using a nullptr out_size_of.
 * @param node Node to get output from
 * @param field Field to get output from. Formatted output with FormatNode should be marked as RGL_FIELD_DYNAMIC_FORMAT.
 * @param out_count Returns the number of available elements (e.g. points). May be null.
 * @param out_size_of Returns byte size of a single element (e.g. point). May be null.
 */
RGL_API rgl_status_t
rgl_graph_get_result_size(rgl_node_t node, rgl_field_t field, int32_t* out_count, int32_t* out_size_of);

/**
 * Obtains the result data of any node in the graph.
 * If the result is not yet available, this function will block.
 * @param node Node to get output from
 * @param field Field to get output from. Formatted output with FormatNode should be marked as RGL_FIELD_DYNAMIC_FORMAT.
 * @param data Returns binary data, expects a buffer of size (*out_count) * (*out_size_of) from rgl_graph_get_result_size(...) call.
 */
RGL_API rgl_status_t
rgl_graph_get_result_data(rgl_node_t node, rgl_field_t field, void* data);

/**
 * Activates or deactivates node in the graph.
 * Children of inactive nodes do not execute as well.
 * Nodes are active by default.
 * @param node Node to activate/deactivate.
 * @param active Whether node active or not.
 */
RGL_API rgl_status_t
rgl_graph_node_set_active(rgl_node_t node, bool active);

/**
 * Adds child to the parent node 
 * @param parent Node that will be set as the parent of (child)
 * @param child Node that will be set as the child of (parent)
 */
RGL_API rgl_status_t
rgl_graph_node_add_child(rgl_node_t parent, rgl_node_t child);

/**
 * Removes child from the parent node
 * @param parent Node that will be removed as parent from (child)
 * @param child Node that will be removed as child from (parent)
 */
RGL_API rgl_status_t
rgl_graph_node_remove_child(rgl_node_t parent, rgl_node_t child);
