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
#define RGL_VERSION_MINOR 14
#define RGL_VERSION_PATCH 1

// Invalid entity ID is assign to rays that does not hit any entity.
// Cannot be assigned to mesh manually. It is reserved for internal raytracing use.
#define RGL_ENTITY_INVALID_ID  0

// Default entity ID is the largest positive 32-bit integer.
// It is assigned by default if the user does not specify it.
#define RGL_DEFAULT_ENTITY_ID 2147483647

/**
 * Two consecutive 32-bit floats.
 */
typedef struct
{
	float value[2];
} rgl_vec2f;

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
 * Right-handed coordinate system.
 */
typedef struct
{
	float value[3][4];
} rgl_mat3x4f;

/**
 * Represents on-GPU mesh that can be referenced by entities on the scene.
 * Each mesh can be referenced by any number of entities on different scenes.
 */
typedef struct Mesh* rgl_mesh_t;

/**
 * Opaque handle representing an object visible to lidars.
 * An entity is always bound to exactly one scene.
 */
typedef struct Entity* rgl_entity_t;

/**
 * Represents on-GPU texture that can be referenced by Entities on the scene.
 * Each texture can be referenced by any number of Entities on different scenes.
 */
typedef struct Texture* rgl_texture_t;

/**
 * TODO(prybicki)
 */
typedef struct Node* rgl_node_t;

/**
 * Opaque handle representing a scene - a collection of entities.
 * Using scene is optional. NULL can be passed to use an implicit default scene.
 */
typedef struct Scene* rgl_scene_t;

/**
 * Enumerates available extensions in RGL which can be queried using `rgl_get_extension_info`.
 * For more details, see the chapter on extensions in the README.
 * The order of constants must not be changed.
 */
typedef enum : int32_t
{
	RGL_EXTENSION_PCL = 0,
	RGL_EXTENSION_ROS2 = 1,
	RGL_EXTENSION_UDP = 2,
	RGL_EXTENSION_COUNT
} rgl_extension_t;

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
typedef enum {
	RGL_FIELD_XYZ_F32 = 1,
	RGL_FIELD_INTENSITY_F32,
	RGL_FIELD_IS_HIT_I32,
	RGL_FIELD_RAY_IDX_U32,
	RGL_FIELD_POINT_IDX_U32,
	RGL_FIELD_ENTITY_ID_I32,
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

/**
 * Helper enum for axis selection
 */
typedef enum
{
	RGL_AXIS_X = 1,
	RGL_AXIS_Y = 2,
	RGL_AXIS_Z = 3,
} rgl_axis_t;

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
 * As stated in README, some RGL features (extensions) are opt-in in compile-time.
 * This call can be used to query in runtime if specific extensions were compiled in the binary.
 * @param extension Extension to query.
 * @param out_available Pointer to the result. Pointee is set to non-zero value if the extension is available.
 */
RGL_API rgl_status_t
rgl_get_extension_info(rgl_extension_t extension, int32_t* out_available);

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
rgl_mesh_create(rgl_mesh_t* out_mesh,
                const rgl_vec3f* vertices,
                int32_t vertex_count,
                const rgl_vec3i* indices,
                int32_t index_count);

/**
 * Assign texture coordinates to given mesh. Pair of texture coordinates is assigned to each vertex.
 *
 * @param mesh Address to store the resulting mesh handle
 * @param uvs An array of rgl_vec2f or binary-compatible data representing mesh uv coordinates
 * @param vertex_count Number of elements in the vertices array. It has to be equal to vertex buffer size.
 */
RGL_API rgl_status_t
rgl_mesh_set_texture_coords(rgl_mesh_t mesh,
                       const rgl_vec2f* uvs,
                       int32_t uv_count);

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
                         const rgl_vec3f* vertices,
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
rgl_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh);

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
rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f *transform);

/**
 * Set instance ID of the given entity.
 * @param entity Entity to modify
 * @param int ID to set. If not set, value of the entity id will be automatically generated as a DEFAULT_ENTITY_ID.
 */
RGL_API rgl_status_t
rgl_entity_set_id(rgl_entity_t entity, int32_t id);

/**
 * Assign intensity texture to the given entity. The assumption is that the entity can hold only one intensity texture.
 * @param entity Entity to modify.
 * @apram texture Texture to assign.
 */
RGL_API rgl_status_t
rgl_entity_set_intensity_texture(rgl_entity_t entity, rgl_texture_t texture);

/******************************* TEXTURE *******************************/

/**
 * Creates a Texture.
 * Texture is a container object which holds device pointer to texture resource.
 * @param out_texture Handle to the created Texture.
 * @param texels Pointer to the texture data. Should be pass as raw byte data of unsigned char array .
 * @param width Width of the texture. Has to be positive.
 * @param height Height of the texture. It is not demanded that width == height. Has to be positive.
 */
RGL_API rgl_status_t
rgl_texture_create(rgl_texture_t* out_texture, const void* texels, int32_t width, int32_t height);

/**
 * Informs that the given texture will be no longer used.
 * The texture will be destroyed after all referring entities are destroyed.
 * @param mesh Texture to be marked as no longer needed
 */
RGL_API rgl_status_t
rgl_texture_destroy(rgl_texture_t texture);

/******************************** SCENE ********************************/

/**
 * Sets time to the given scene.
 * Time indicates a specific point in the simulation timeline when raytrace is performed.
 * Timestamp is used to fill field RGL_FIELD_TIME_STAMP_F64 or for ROS2 publishing.
 * @param scene Scene where time will be set. Pass NULL to use the default scene.
 * @param nanoseconds Timestamp in nanoseconds.
 */
RGL_API rgl_status_t
rgl_scene_set_time(rgl_scene_t scene, uint64_t nanoseconds);

/******************************** NODES ********************************/

/**
 * Creates or modifies FromMat3x4fRaysNode.
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
 * Creates or modifies SetRingIdsRaysNode.
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
 * Creates or modifies SetRangeRaysNode.
 * The node defines min and max range for existing rays.
 * It could be passed:
 * - one range value and it will be applied to all of the rays
 * - number of ranges equals to the number of rays (range for each ray will be specified individually)
 * Input: rays
 * Output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param ranges Pointer to ranges.
 * @param ranges_count Size of the `ranges` array.
 */
RGL_API rgl_status_t
rgl_node_rays_set_range(rgl_node_t* node, const rgl_vec2f* ranges, int32_t ranges_count);

/**
 * Creates or modifies SetTimeOffsetsRaysNode.
 * The node assigns time offsets for existing rays.
 * Input: rays
 * Output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param offsets Pointer to time offsets. Time offsets are in milliseconds.
 * @param offsets_count Size of the `offsets` array. It has to be equal to number of existing rays.
 */
RGL_API rgl_status_t
rgl_node_rays_set_time_offsets(rgl_node_t* node, const float* offsets, int32_t offsets_count);

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

/**
 * Creates or modifies VelocityDistortRaysNode.
 * The node distorts rays according to the sensor's linear velocity and angular velocity.
 * NOTE!
 * The velocities passed to that node must match the coordinate frame in which rays are described/transformed.
 * If node is before TransformRaysNode, then the velocities must be in the sensor-local coordinate frame.
 * Otherwise, user hase to transform the velocities to TransformRaysNode coordinate frame.
 * This node requires that rays time offsets are set.
 * The distortion takes into account only sensor velocity.
 * The velocity of the objects being scanned by the sensor is not considered.
 * Graph input: rays
 * Graph output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param velocity velocity Pointer to a single 3D vector describing the linear velocity of the sensor. The velocity is in meters per second.
 * @param angularVelocity Pointer to a single 3D vector describing the delta angular velocity  of the sensor in euler angles (roll, pitch, yaw). The velocity is in radians per second.
 */
RGL_API rgl_status_t
rgl_node_rays_velocity_distort(rgl_node_t* node, const rgl_vec3f* linear_velocity, const rgl_vec3f* angular_velocity);

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
 */
RGL_API rgl_status_t
rgl_node_raytrace(rgl_node_t* node, rgl_scene_t scene);

/**
 * Creates or modifies FormatPointsNode.
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
 * Creates or modifies CompactPointsNode.
 * The node removes non-hit points. In other words, it converts a point cloud into a dense one.
 * Graph input: point cloud
 * Graph output: point cloud (compacted)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 */
RGL_API rgl_status_t
rgl_node_points_compact(rgl_node_t* node);

/**
 * Creates or modifies SpatialMergePointsNode.
 * The node merges point clouds spatially (e.g., multiple lidars outputs into one point cloud).
 * Only provided fields are merged (RGL_FIELD_DYNAMIC_FORMAT is not supported).
 * Input point clouds must be unorganized (height == 1).
 * Any modification to the node's parameters clears accumulated data.
 * Graph input: point cloud(s)
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param fields Fields to be merged.
 * @param field_count Number of elements in the `fields` array.
 */
RGL_API rgl_status_t
rgl_node_points_spatial_merge(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies TemporalMergePointsNode.
 * The node accumulates (performs temporal merge on) point clouds on each run.
 * Only provided fields are merged (RGL_FIELD_DYNAMIC_FORMAT is not supported).
 * Input point cloud must be unorganized (height == 1).
 * Any modification to the node's parameters clears accumulated data.
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param fields Fields to be merged.
 * @param field_count Number of elements in the `fields` array.
 */
RGL_API rgl_status_t
rgl_node_points_temporal_merge(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies FromArrayPointsNode.
 * The node provides initial points for its children nodes.
 * Input: none
 * Output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param points Pointer to the array of points. Point is represented as a structure composed of fields.
 * See RGLFields.hpp (https://github.com/RobotecAI/RobotecGPULidar/blob/main/src/RGLFields.hpp).
 * Example of that structure:
 * struct ExamplePoint
 * {
 *   Field<XYZ_F32>::type xyz;
 *   Field<PADDING_32>::type padding;
 *   Field<IS_HIT_I32>::type isHit;
 *   Field<INTENSITY_F32>::type intensity;
 * };
 * @param points_count Number of elements in the `points` array.
 * @param rgl_field_t Subsequent fields to be present in the binary input.
 * @param field_count Number of elements in the `fields` array.
 */
RGL_API rgl_status_t
rgl_node_points_from_array(rgl_node_t* node, const void* points, int32_t points_count, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies GaussianNoiseAngularRayNode.
 * Applies angular noise to the rays before raycasting.
 * See documentation: https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md#ray-based-angular-noise
 * Graph input: rays
 * Graph output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param mean Angular noise mean in radians.
 * @param st_dev Angular noise standard deviation in radians.
 * @param axis Axis on which angular noise will be perform.
 */
RGL_API rgl_status_t
rgl_node_gaussian_noise_angular_ray(rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis);

/**
 * Creates or modifies GaussianNoiseAngularHitpointNode.
 * Adds angular noise to already computed hitpoints.
 * Note: affects on RGL_FIELD_XYZ_F32 and RGL_DISTANCE_F32.
 * Should be used after raytrace node.
 * Using this noise after nodes that modify XYZ (e.g. points_transform, points_downsample) may cause incorrect values in fields other than RGL_FIELD_XYZ_F32.
 * See documentation: https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md#hitpoint-based-angular-noise
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param mean Angular noise mean in radians.
 * @param st_dev Angular noise standard deviation in radians.
 * @param axis Axis on which angular noise will be perform.
 */
RGL_API rgl_status_t
rgl_node_gaussian_noise_angular_hitpoint(rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis);

/**
 * Creates or modifies GaussianNoiseDistanceNode.
 * Changes the distance between hitpoint and lidar's origin.
 * Note: affects on RGL_FIELD_XYZ_F32 and RGL_DISTANCE_F32.
 * Should be used after raytrace node.
 * Using this noise after nodes that modify XYZ (e.g. points_transform, points_downsample) may cause incorrect values in fields other than RGL_FIELD_XYZ_F32.
 * See documentation: https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md#distance-noise
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param mean Distance noise mean in meters.
 * @param st_dev_base Distance noise standard deviation base in meters.
 * @param st_dev_rise_per_meter Distance noise standard deviation rise per meter.
 */
RGL_API rgl_status_t
rgl_node_gaussian_noise_distance(rgl_node_t* node, float mean, float st_dev_base, float st_dev_rise_per_meter);

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
