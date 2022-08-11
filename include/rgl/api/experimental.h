#pragma once

#include <macros/visibility.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RGL_VERSION_MAJOR 0
#define RGL_VERSION_MINOR 11
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
 * Errors that are not specified explicitly as recoverable
 * should be considered unrecoverable, e.g. requiring reloading the library (restarting the application).
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
	 * This is a recoverable error.
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
	 * Indicates an error in the pipeline, such as adjacency of incompatible nodes.
	 */
	RGL_INVALID_PIPELINE,

	/**
	 * Requested functionality has been not yet implemented.
	 * This is a recoverable error.
	 */
	RGL_NOT_IMPLEMENTED = 404,

	/**
	 * An unhandled internal error has occurred.
	 * If you see this error, please file a bug report.
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
 * TODO(prybicki)
 */
typedef enum
{
	RGL_FIELD_X_F32,
	RGL_FIELD_Y_F32,
	RGL_FIELD_Z_F32,
	RGL_FIELD_INTENSITY_F32,
	RGL_FIELD_RING_ID_U16,
	RGL_FIELD_AZIMUTH_F32,
	RGL_FIELD_DISTANCE_F32,
	RGL_FIELD_RETURN_TYPE_U8,
	RGL_FIELD_TIME_STAMP_F64,
	RGL_FIELD_PADDING_8,
	RGL_FIELD_PADDING_16,
	RGL_FIELD_PADDING_32,
	// ...
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
rgl_get_version_info(int *out_major, int *out_minor, int *out_patch);

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
 * @param vertices An array of rgl_vec3i or binary-compatible data representing mesh indices
 * @param index_count Number of elements in the indices array
 */
RGL_API rgl_status_t
rgl_mesh_create(rgl_mesh_t *out_mesh,
                const rgl_vec3f *vertices,
                int vertex_count,
                const rgl_vec3i *indices,
                int index_count);

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
                         int vertex_count);


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

// /******************************** PIPELINE ********************************/

RGL_API rgl_status_t
rgl_pipeline_run(rgl_node_t node);

RGL_API rgl_status_t
rgl_pipeline_use_rays_mat3x4f(rgl_node_t* node, rgl_node_t parent, rgl_mat3x4f* rays, size_t ray_count);

RGL_API rgl_status_t
rgl_pipeline_raytrace(rgl_node_t* node, rgl_node_t parent, rgl_scene_t scene, float range);

RGL_API rgl_status_t
rgl_pipeline_write_pcd_file(rgl_node_t* node, rgl_node_t parent, const char* file_path);

// Cuts out requested fields and formats a contiguous binary buffer.
RGL_API rgl_status_t
rgl_pipeline_format(rgl_node_t* node, rgl_node_t parent, rgl_field_t* fields, int field_count);

// Removes non-hit points. Performed lazily - only one occurrence in the pipeline will have computational cost.
RGL_API rgl_status_t
rgl_pipeline_compact(rgl_node_t* node, rgl_node_t parent);

// Reduces the number of points using the PCL library.
RGL_API rgl_status_t
rgl_pipeline_downsample(rgl_node_t* node, rgl_node_t parent, float leaf_size);

// Applies affine transformation, e.g. to change the coordinate frame.
RGL_API rgl_status_t
rgl_pipeline_transform_pointcloud(rgl_node_t* node, rgl_node_t parent, rgl_mat3x4f transform);

RGL_API rgl_status_t
rgl_pipeline_transform_rays(rgl_node_t* node, rgl_node_t parent, rgl_mat3x4f transform);


// Applies gaussian noise.
// RGL_API rgl_status_t
// rgl_pipeline_apply_gaussian_noise(rgl_node_t* out_node, rgl_node_t parent,
//                                   rgl_angular_noise_type_t angular_noise_type,
//                                   float angular_noise_stddev,
//                                   float angular_noise_mean,
//                                   float distance_noise_stddev_base,
//                                   float distance_noise_stddev_rise_per_meter,
//                                   float distance_noise_mean);

// Appends data from the parent node to the given PCD file
RGL_API rgl_status_t
rgl_pipeline_write_pcd_file(rgl_node_t* node, rgl_node_t parent, const char* file_path);

// Publishes data from the parent node on the given topic
RGL_API rgl_status_t
rgl_pipeline_publish_ros2_topic(rgl_node_t channel, rgl_node_t parent, const char* topic /* QoS settings, etc */);

// // Returns the minimal size of the buffer required to receive data from the leaf node of a pipeline
// RGL_API rgl_status_t
// rgl_pipeline_node_get_results_size(rgl_lidar_t lidar, rgl_node_t node, int* point_count);
//
// // Returns binary data from the given leaf node of a pipeline.
// RGL_API rgl_status_t
// rgl_pipeline_node_get_results_data(rgl_lidar_t lidar, rgl_node_t node, void* data);
//
// // Requests LiDAR to perform the pipeline after raytracing.
// // Point dimensions to be computed are inferred from all format nodes in the pipeline.
// RGL_API rgl_status_t
// rgl_lidar_set_pipeline(rgl_lidar_t lidar, rgl_node_t pipeline_root_node);