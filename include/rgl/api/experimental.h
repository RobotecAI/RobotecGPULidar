#pragma once

#include <macros/visibility.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Three consecutive 32-bit floats.
 */
typedef struct
{
	float value[3];
} rgl_vec3f;

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
 * Opaque handle representing a Lidar, including
 * - it's position
 * - rays' starting positions and directions, relative to the lidar,
 * - rays' ranges
 * - possibly other properties in the future
 * Note: Creating and destroying a Lidar is a costly operation.
 */
typedef struct Lidar *rgl_lidar_t;

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
 * Output data formats.
 */
typedef enum
{
	RGL_FORMAT_INVALID = 0,
	/**
	 * Three consecutive 32-bit floats describing hit-point coordinates in the world frame. Does not contain non-hits.
	 */
	RGL_FORMAT_XYZ = 1,
	RGL_FORMAT_COUNT
} rgl_format_t;

/******************************** GENERAL ********************************/

/**
 * Returns data describing semantic version as described in https://semver.org/
 * Version string can be obtained by formatting "{out_major}.{out_minor}.{out_patch}{out_suffix}.
 * Hash is provided mostly for debugging and issue reporting.
 * @param out_major Address to store major version number
 * @param out_minor Address to store minor version number
 * @param out_patch Address to store patch version number
 */
RGL_API rgl_status_t
rgl_get_version_info(int *out_major, int *out_minor, int *out_patch);

/**
 * Returns pointer to string explaining the most recent error. This function always succeeds.
 * @param out_error Address to store pointer to string explaining the cause of the given error.
 * Returned pointer is valid only until next RGL API call.
 */
RGL_API void
rgl_get_last_error_string(const char **out_error_string);

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
                rgl_vec3f *vertices,
                int vertex_count,
                rgl_vec3i *indices,
                int index_count);

/**
 * Informs that the given mesh will be no longer used.
 * The mesh will be destroyed after all referring entities are destroyed.
 * @param mesh Mesh to be marked as no longer needed
 */
RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh);

/**
 * Updates mesh vertex data. Note: if indices were set, then the number of vertices must not change.
 * @param mesh Mesh to modify
 * @param vertices An array of rgl_vec3f or binary-compatible data representing mesh vertices
 * @param vertex_count Number of (rgl_vec3f) elements of the vertices array
 */
RGL_API rgl_status_t
rgl_mesh_set_vertices(rgl_mesh_t mesh,
                      rgl_vec3f *vertices,
                      int vertex_count);


/******************************** ENTITY ********************************/

/**
 * Creates an entity and adds it to the given scene.
 * Entity is a lightweight object which pairs a reference to a mesh with a 3D affine transform.
 * NOTE: Current implementation requires each entity to use a unique mesh handle. This limitation will be removed in the future.
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
rgl_entity_set_pose(rgl_entity_t entity, rgl_mat3x4f *local_to_world_tf);

/******************************** LIDAR ********************************/

/**
 * Creates lidar from ray transforms.
 * Lidar range will be set to default (inf).
 * @param out_lidar Address where to store resulting lidar handle.
 */
RGL_API rgl_status_t
rgl_lidar_create(rgl_lidar_t *out_lidar,
                 rgl_mat3x4f *ray_transforms,
                 int ray_transforms_count);

RGL_API rgl_status_t
rgl_lidar_create_with_frames(rgl_lidar_t* lidar,
                             rgl_mat3x4f *ray_transforms,
                             int ray_transforms_count,
                             int* frame_sizes,
                             int frame_count);

RGL_API rgl_status_t
rgl_lidar_next_frame(rgl_lidar_t lidar);

/**
 * Allows to set maximum distance that rays are to be traced.
 * Hit-points further than given range will be not reported.
 * @param lidar Lidar handle to change range.
 * @param range Positive, finite real number.
 */
RGL_API rgl_status_t
rgl_lidar_set_range(rgl_lidar_t lidar, float range);

/**
 * Destroys lidar and releases its resources. After this call, provided lidar handle must not be used.
 * @param lidar Lidar handle to be destroyed.
 */
RGL_API rgl_status_t
rgl_lidar_destroy(rgl_lidar_t lidar);

/**
 * Changes position of the lidar, i.e. base transform transform for all rays.
 * @param lidar Lidar handle to have the pose changed
 * @param transform Pointer to rgl_mat3x4f (or binary-compatible data) representing desired (entity -> world) coordinate system transform.
 */
RGL_API rgl_status_t
rgl_lidar_set_pose(rgl_lidar_t lidar, rgl_mat3x4f *local_to_world_tf);

/**
 * Initiates raytracing for the given scene and lidar and returns immediately.
 * When raytracing is in-progress, some calls may be blocking.
 * @param scene Handle of the scene where raytracing will be performed. Pass NULL to use the default scene.
 * @param lidar Handle of the lidar that will be used to perform raytracing.
 */
RGL_API rgl_status_t
rgl_lidar_raytrace_async(rgl_scene_t scene, rgl_lidar_t lidar);

/**
 * Returns number of elements (e.g. hit-points) of the most recent result of the given format.
 * User should prepare a buffer at least <number-of-elements> * <size-of-element> bytes long.
 * @param format Format to query
 * @param out_size Address to store result
 */
RGL_API rgl_status_t
rgl_lidar_get_output_size(rgl_lidar_t lidar, int *out_size);

/**
 * Copy results of the given format into provided buffer.
 * Required buffer size can be queried using rgl_lidar_get_output_size.
 * @param format Result format to copy
 * @param out_data Target buffer
 */
RGL_API rgl_status_t
rgl_lidar_get_output_data(rgl_lidar_t lidar, rgl_format_t format, void *out_data);
