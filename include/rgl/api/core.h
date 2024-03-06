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
#define RGL_VISIBLE __attribute__((dllexport))
#else
#define RGL_VISIBLE __declspec(dllexport)
#endif // __GNUC__
#else
#define RGL_VISIBLE __attribute__((visibility("default")))
#if __GNUC__ >= 4
#define RGL_VISIBLE __attribute__((visibility("default")))
#else
#define RGL_VISIBLE
#endif
#endif // _WIN32 || __CYGWIN__

#define RGL_API NO_MANGLING RGL_VISIBLE

#define RGL_VERSION_MAJOR 0
#define RGL_VERSION_MINOR 16
#define RGL_VERSION_PATCH 3

// Invalid Entity ID is assign to rays that does not hit any Entity.
// Cannot be assigned to Mesh manually. It is reserved for internal raytracing use.
#define RGL_ENTITY_INVALID_ID 0

// Default Entity ID is the largest positive 28-bit integer (OptiX limit).
// It is assigned by default if the user does not specify it.
#define RGL_DEFAULT_ENTITY_ID 268435455

/**
 * Two consecutive 32-bit floats.
 */
typedef struct
{
	float value[2];
} rgl_vec2f;

#ifndef __cplusplus
static_assert(sizeof(rgl_vec2f) == 2 * sizeof(float));
static_assert(std::is_trivial_v<rgl_vec2f>);
static_assert(std::is_standard_layout_v<rgl_vec2f>);
#endif

/**
 * Three consecutive 32-bit floats.
 */
typedef struct
{
	float value[3];
} rgl_vec3f;

#ifndef __cplusplus
static_assert(sizeof(rgl_vec3f) == 3 * sizeof(float));
static_assert(std::is_trivial_v<rgl_vec3f>);
static_assert(std::is_standard_layout_v<rgl_vec3f>);
#endif

/**
 * Three consecutive 32-bit signed integers.
 */
typedef struct
{
	int32_t value[3];
} rgl_vec3i;

#ifndef __cplusplus
static_assert(sizeof(rgl_vec3i) == 3 * sizeof(int32_t));
static_assert(std::is_trivial_v<rgl_vec3i>);
static_assert(std::is_standard_layout_v<rgl_vec3i>);
#endif

/**
 * Row-major matrix with 3 rows and 4 columns of 32-bit floats.
 * Right-handed coordinate system.
 */
typedef struct
{
	float value[3][4];
} rgl_mat3x4f;

#ifndef __cplusplus
static_assert(sizeof(rgl_mat3x4f) == 3 * 4 * sizeof(float));
static_assert(std::is_trivial_v<rgl_mat3x4f>);
static_assert(std::is_standard_layout_v<rgl_mat3x4f>);
#endif

/**
 * Radar parameters applied at a given distance range.
 */
typedef struct
{
	/**
	 * The beginning distance range for the parameters.
	 */
	float begin_distance;
	/**
	 * The end range distance for the parameters.
	 */
	float end_distance;
	/**
	 * The maximum distance difference to create a new radar detection (in simulation units).
	 */
	float distance_separation_threshold;
	/**
	 * The maximum radial speed difference to create a new radar detection (in simulation units)
	 */
	float radial_speed_separation_threshold;
	/**
	 * The maximum azimuth difference to create a new radar detection (in radians).
	 */
	float azimuth_separation_threshold;
} rgl_radar_scope_t;

#ifndef __cplusplus
static_assert(sizeof(rgl_radar_separations_t) == 5 * sizeof(float));
static_assert(std::is_trivial_v<rgl_radar_separations_t>);
static_assert(std::is_standard_layout_v<rgl_radar_separations_t>);
#endif

/**
 * Represents on-GPU Mesh that can be referenced by Entities on the Scene.
 * Each Mesh can be referenced by any number of Entities on different Scenes.
 */
typedef struct Mesh* rgl_mesh_t;

/**
 * Opaque handle representing an object visible to lidars.
 * An Entity is always bound to exactly one Scene.
 */
typedef struct Entity* rgl_entity_t;

/**
 * Represents on-GPU texture that can be referenced by Entities on the Scene.
 * Each texture can be referenced by any number of Entities on different Scenes.
 */
typedef struct Texture* rgl_texture_t;

/**
 * Opaque handle for a computational graph node in RGL.
 * Represents a single computational step, e.g. introducing gaussian noise, raytracing or downsampling.
 * Nodes form a directed acyclic graph, dictating execution order.
 */
typedef struct Node* rgl_node_t;

/**
 * Opaque handle representing a Scene - a collection of Entities.
 * Using Scene is optional. NULL can be passed to use an implicit default Scene.
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
	RGL_EXTENSION_SNOW = 3,
	RGL_EXTENSION_COUNT
} rgl_extension_t;

/**
 * Status (error) codes returned by all RGL API functions.
 * Unrecoverable errors require reloading the library (restarting the application).
 */
typedef enum : int32_t
{
	/**
	 * Operation successful.
	 * This is a recoverable error :)
	 */
	RGL_SUCCESS = 0,

	/**
	 * One of the arguments is invalid (e.g., null pointer) or a number in an invalid range.
	 * Get the error string for more details.
	 * This is a recoverable error.
	 */
	RGL_INVALID_ARGUMENT,

	/**
	 * RGL's internal state has been corrupted by a previous unrecoverable error.
	 * Application must be restarted.
	 */
	RGL_INVALID_STATE,

	/**
	 * Indicates that a logging operation (e.g., configuration) was unsuccessful.
	 * This is an unrecoverable error.
	 */
	RGL_LOGGING_ERROR,

	/**
	 * Indicates that provided API object handle is not known by RGL.
	 * This can be caused by using previously destroyed API objects, e.g.
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
	* Indicates that a UDP operation (e.g., creating socket, sending packet) was not successful.
	* This is a recoverable error.
	*/
	RGL_UDP_ERROR,

	/**
	* Indicates that a ROS2 native library throws an exception.
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
	 * Requested functionality still needs to be implemented.
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
typedef enum : int32_t
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
 * Available point attributes used to specify the layout of the binary data.
 */
typedef enum : int32_t
{
	RGL_FIELD_XYZ_VEC3_F32 = 1,
	RGL_FIELD_INTENSITY_F32,
	RGL_FIELD_IS_HIT_I32,
	RGL_FIELD_IS_GROUND_I32,
	RGL_FIELD_RAY_IDX_U32,
	RGL_FIELD_ENTITY_ID_I32,
	RGL_FIELD_DISTANCE_F32,
	RGL_FIELD_AZIMUTH_F32,   // In radians. Assuming up vector is Y, forward vector is Z.
	RGL_FIELD_ELEVATION_F32, // In radians. Assuming up vector is Y, forward vector is Z.
	RGL_FIELD_RING_ID_U16,
	RGL_FIELD_RETURN_TYPE_U8,
	RGL_FIELD_TIME_STAMP_F64,

	/**
	 * Velocity of the hit point on the entity.
	 * It depends on entity's
	 * - linear velocity
	 * - angular velocity
	 * - mesh deformations (e.g. skinning)
	 * The aforementioned are inferred from calls to `rgl_entity_set_pose`, `rgl_scene_set_time` and `rgl_mesh_update_vertices`.
	 */
	RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32,

	/**
	 * Velocity of the hit point on the entity, in the coordinate frame of rays source.
	 */
	RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32,

	/**
	 * Scalar describing distance increase per second between hit point and ray source.
	 */
	RGL_FIELD_RADIAL_SPEED_F32,

	/**
	 * Radar-specific fields. At the moment, placeholders only to implement in the future.
	 */
	RGL_FIELD_POWER_F32,
	RGL_FIELD_RCS_F32, // Radar cross-section
	RGL_FIELD_NOISE_F32,
	RGL_FIELD_SNR_F32, // Signal-to-noise ratio

	/**
	 * Normal vector of the mesh triangle where the hit-point is located.
	 * Assumes right-hand rule of vertices ordering.
	 */
	RGL_FIELD_NORMAL_VEC3_F32,

	/**
	 * Incident angle of the ray hitting the mesh triangle in radians.
	 * In range [0, PI/2) rad, where 0 means the ray hit the triangle perpendicularly.
	 */
	RGL_FIELD_INCIDENT_ANGLE_F32,

	/**
	 * 3x4 matrix describing pose of the ray in the world coordinate system.
	 */
	RGL_FIELD_RAY_POSE_MAT3x4_F32,

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
typedef enum : int32_t
{
	RGL_AXIS_X = 1,
	RGL_AXIS_Y = 2,
	RGL_AXIS_Z = 3,
} rgl_axis_t;

/******************************** GENERAL ********************************/

/**
 * Returns data describing semantic version as described in https://semver.org/
 * Version string can be obtained by formatting "{out_major}.{out_minor}.{out_patch}".
 * Hash is provided mainly for debugging and issue reporting.
 * @param out_major Address to store major version number
 * @param out_minor Address to store minor version number
 * @param out_patch Address to store patch version number
 */
RGL_API rgl_status_t rgl_get_version_info(int32_t* out_major, int32_t* out_minor, int32_t* out_patch);

/**
 * As stated in README, some RGL features (extensions) are opt-in in compile-time.
 * This call can be used to query in runtime if specific extensions were compiled in the binary.
 * @param extension Extension to query.
 * @param out_available Pointer to the result. Pointee is set to non-zero value if the extension is available.
 */
RGL_API rgl_status_t rgl_get_extension_info(rgl_extension_t extension, int32_t* out_available);

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
RGL_API rgl_status_t rgl_configure_logging(rgl_log_level_t log_level, const char* log_file_path, bool use_stdout);

/**
 * Returns a pointer to a string explaining the last error. This function always succeeds.
 * Returned pointer is valid only until the next RGL API call.
 * @param out_error Address to store a pointer to the string explaining the error's cause.
 */
RGL_API void rgl_get_last_error_string(const char** out_error_string);

/**
 * Removes all user-created API objects: Meshes, Entities, Scenes, lidars, etc.
 * Effectively brings the library to the state as if it was not yet used.
 * All API handles are invalidated.
 */
RGL_API rgl_status_t rgl_cleanup(void);

/******************************** MESH ********************************/

/**
 * Creates Mesh from the arrays of vertices and indices. CW/CCW order does not matter.
 * Provided arrays are copied to the GPU before this function returns.
 * @param out_mesh Address to store the resulting Mesh handle
 * @param vertices An array of rgl_vec3f or binary-compatible data representing Mesh vertices
 * @param vertex_count Number of elements in the vertices array
 * @param indices An array of rgl_vec3i or binary-compatible data representing Mesh indices
 * @param index_count Number of elements in the indices array
 */
RGL_API rgl_status_t rgl_mesh_create(rgl_mesh_t* out_mesh, const rgl_vec3f* vertices, int32_t vertex_count,
                                     const rgl_vec3i* indices, int32_t index_count);

/**
 * Assign texture coordinates to given Mesh. Pair of texture coordinates is assigned to each vertex.
 *
 * @param mesh Address to store the resulting Mesh handle
 * @param uvs An array of rgl_vec2f or binary-compatible data representing Mesh uv coordinates
 * @param vertex_count Number of elements in the vertices array. It has to be equal to vertex buffer size.
 */
RGL_API rgl_status_t rgl_mesh_set_texture_coords(rgl_mesh_t mesh, const rgl_vec2f* uvs, int32_t uv_count);

/**
 * Informs that the given Mesh will be no longer used.
 * The Mesh will be destroyed after all referring Entities are destroyed.
 * @param mesh Mesh to be marked as no longer needed
 */
RGL_API rgl_status_t rgl_mesh_destroy(rgl_mesh_t mesh);

/**
 * Updates Mesh vertex data. The number of vertices must not change.
 * This function is intended to update animated Meshes.
 * Should be called after rgl_scene_set_time to ensure proper velocity computation.
 * @param mesh Mesh to modify
 * @param vertices An array of rgl_vec3f or binary-compatible data representing Mesh vertices
 * @param vertex_count Number of elements in the vertices array. It must be equal to the original vertex count!
 */
RGL_API rgl_status_t rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int32_t vertex_count);

/******************************** ENTITY ********************************/

/**
 * Creates an Entity and adds it to the given Scene.
 * Entity is a lightweight object that pairs a Mesh reference with a 3D affine transform.
 * @param out_entity Handle to the created Entity.
 * @param scene Scene where the Entity will be added. Pass NULL to use the default Scene.
 * @param mesh Handle to the Mesh, which will represent the Entity on the Scene.
 */
RGL_API rgl_status_t rgl_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh);

/**
 * Removes an Entity from the Scene and releases its resources (memory).
 * This operation does not affect the Entity's Mesh since it can be shared among other Entities.
 * @param entity Entity to remove
 */
RGL_API rgl_status_t rgl_entity_destroy(rgl_entity_t entity);

/**
 * Changes transform (position, rotation, scaling) of the given Entity.
 * Should be called after rgl_scene_set_time to ensure proper velocity computation.
 * @param entity Entity to modify
 * @param transform Pointer to rgl_mat3x4f (or binary-compatible data) representing desired (Entity -> world) coordinate system transform.
 */
RGL_API rgl_status_t rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f* transform);

/**
 * Set instance ID of the given Entity.
 * @param entity Entity to modify
 * @param int ID to set. If not set, value of the Entity id will be automatically generated as a DEFAULT_ENTITY_ID.
 */
RGL_API rgl_status_t rgl_entity_set_id(rgl_entity_t entity, int32_t id);

/**
 * Assign intensity texture to the given Entity. The assumption is that the Entity can hold only one intensity texture.
 * @param entity Entity to modify.
 * @apram texture Texture to assign.
 */
RGL_API rgl_status_t rgl_entity_set_intensity_texture(rgl_entity_t entity, rgl_texture_t texture);

/******************************* TEXTURE *******************************/

/**
 * Creates a Texture.
 * Texture is a container object which holds device pointer to texture resource.
 * @param out_texture Handle to the created Texture.
 * @param texels Pointer to the texture data. Should be pass as raw byte data of unsigned char array .
 * @param width Width of the texture. Has to be positive.
 * @param height Height of the texture. It is not demanded that width == height. Has to be positive.
 */
RGL_API rgl_status_t rgl_texture_create(rgl_texture_t* out_texture, const void* texels, int32_t width, int32_t height);

/**
 * Informs that the given texture will be no longer used.
 * The texture will be destroyed after all referring Entities are destroyed.
 * @param mesh Texture to be marked as no longer needed
 */
RGL_API rgl_status_t rgl_texture_destroy(rgl_texture_t texture);

/******************************** SCENE ********************************/

/**
 * Sets time for the given Scene.
 * Time indicates a specific point when the ray trace is performed in the simulation timeline.
 * Calling this function before updating entities/meshes is required to compute velocity of the hit points.
 * Timestamp may be also used to fill field RGL_FIELD_TIME_STAMP_F64 or for ROS2 publishing.
 * @param scene Scene where time will be set. Pass NULL to use the default Scene.
 * @param nanoseconds Timestamp in nanoseconds.
 */
RGL_API rgl_status_t rgl_scene_set_time(rgl_scene_t scene, uint64_t nanoseconds);

/******************************** NODES ********************************/

/**
 * Creates or modifies FromMat3x4fRaysNode.
 * The Node provides initial rays for its children Nodes.
 * Initial rays are usually provided in the device-local coordinate frame, i.e., close to (0, 0, 0).
 * Input: none
 * Output: rays
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param rays Pointer to 3x4 affine matrices describing rays poses.
 * @param ray_count Size of the `rays` array
 */
RGL_API rgl_status_t rgl_node_rays_from_mat3x4f(rgl_node_t* node, const rgl_mat3x4f* rays, int32_t ray_count);

/**
 * Creates or modifies SetRingIdsRaysNode.
 * The Node assigns ring ids for existing rays.
 * Input: rays
 * Output: rays
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param ring_ids Pointer to ring ids.
 * @param ray_count Size of the `ring_ids` array.
 */
RGL_API rgl_status_t rgl_node_rays_set_ring_ids(rgl_node_t* node, const int32_t* ring_ids, int32_t ring_ids_count);

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
RGL_API rgl_status_t rgl_node_rays_set_range(rgl_node_t* node, const rgl_vec2f* ranges, int32_t ranges_count);

/**
 * Creates or modifies SetTimeOffsetsRaysNode.
 * The node assigns time offsets for existing rays.
 * Input: rays
 * Output: rays
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param offsets Pointer to time offsets. Time offsets are in milliseconds.
 * @param offsets_count Size of the `offsets` array. It has to be equal to number of existing rays.
 */
RGL_API rgl_status_t rgl_node_rays_set_time_offsets(rgl_node_t* node, const float* offsets, int32_t offsets_count);

/**
 * Creates or modifies TransformRaysNode.
 * Effectively, the Node performs the following operation for all rays: `outputRay[i] = (*transform) * inputRay[i]`
 * This function can be used to account for the pose of the device.
 * Graph input: rays
 * Graph output: rays
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param transform Pointer to a single 3x4 affine matrix describing the transformation to be applied.
 */
RGL_API rgl_status_t rgl_node_rays_transform(rgl_node_t* node, const rgl_mat3x4f* transform);

/**
 * Creates or modifies TransformPointsNode.
 * The Node applies the affine transformation to all points.
 * It can be used to e.g., change coordinate frame after raytracing.
 * Note: affects only RGL_FIELD_XYZ_VEC3_F32. Other fields are not modified.
 * Graph input: point cloud
 * Graph output: point cloud (transformed)
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param transform Pointer to a single 3x4 affine matrix describing the transformation to be applied.
 */
RGL_API rgl_status_t rgl_node_points_transform(rgl_node_t* node, const rgl_mat3x4f* transform);

// TODO: remove scene parameter here and in other API calls
/**
 * Creates or modifies RaytraceNode.
 * The Node performs GPU-accelerated raytracing on the given Scene.
 * Fields to be computed will be automatically determined based on connected FormatNodes and YieldPointsNodes
 * Graph input: rays
 * Graph output: point cloud (sparse)
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param scene Handle to a Scene to perform raytracing on. Pass null to use the default Scene
 */
RGL_API rgl_status_t rgl_node_raytrace(rgl_node_t* node, rgl_scene_t scene);

// TODO: Remove this API call on a new release. It is replaced by `rgl_node_raytrace_in_motion`.
/**
 * Creates or modifies RaytraceNode.
 * The same as rgl_node_raytrace, but it applies velocity distortion additionally.
 * To perform raytrace with velocity distortion the time offsets must be set to the rays (using rgl_node_rays_set_time_offsets).
 * The velocities passed to that node must be in the local coordinate frame in which rays are described.
 * NOTE:
 * The distortion takes into account only sensor velocity. The velocity of the objects being scanned by the sensor is not considered.
 * Graph input: rays
 * Graph output: point cloud (sparse)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param scene Handle to a scene to perform raytracing on. Pass null to use the default scene
 * @param linear_velocity Pointer to a single 3D vector describing the linear velocity of the sensor.
 *                        The velocity is in units per second.
 * @param angular_velocity Pointer to a single 3D vector describing the delta angular velocity of the sensor in euler angles (roll, pitch, yaw).
 *                         The velocity is in radians per second.
 */
RGL_API rgl_status_t rgl_node_raytrace_with_distortion(rgl_node_t* node, rgl_scene_t scene, const rgl_vec3f* linear_velocity,
                                                       const rgl_vec3f* angular_velocity);

/**
 * Creates or modifies RaytraceNode.
 * The same as rgl_node_raytrace, but it provides sensor's velocity to the pipeline.
 * It is required to perform velocity distortion or calculate fields: RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32 and RGL_FIELD_RADIAL_SPEED_F32
 *
 * Velocity distortion:
 * To perform raytrace with velocity distortion the time offsets must be set to the rays (using rgl_node_rays_set_time_offsets).
 * The velocities passed to that node must be in the local coordinate frame in which rays are described.
 * NOTE:
 * The distortion takes into account only sensor velocity. The velocity of the objects being scanned by the sensor is not considered.
 *
 * Relative velocity calculation:
 * To calculate relative velocity the pipeline must allow to compute absolute velocities. For more details refer to API calls documentation:
 * `rgl_scene_set_time`, `rgl_entity_set_pose`, and `rgl_mesh_update_vertices`
 *
 * Graph input: rays
 * Graph output: point cloud (sparse)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param scene Handle to a scene to perform raytracing on. Pass null to use the default scene
 * @param linear_velocity Pointer to a single 3D vector describing the linear velocity of the sensor.
 *                        The velocity is in units per second.
 * @param angular_velocity Pointer to a single 3D vector describing the delta angular velocity of the sensor in euler angles (roll, pitch, yaw).
 *                         The velocity is in radians per second.
 * @param apply_ray_distortion If true, velocity distortion feature will be enabled.
 */
RGL_API rgl_status_t rgl_node_raytrace_in_motion(rgl_node_t* node, rgl_scene_t scene, const rgl_vec3f* linear_velocity,
                                                 const rgl_vec3f* angular_velocity, bool apply_ray_distortion);

/**
 * Creates or modifies FormatPointsNode.
 * The Node converts internal representation into a binary format defined by the `fields` array.
 * Note: It is the user's responsibility to ensure proper data structure alignment. See (https://en.wikipedia.org/wiki/Data_structure_alignment).
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param fields Subsequent fields to be present in the binary output
 * @param field_count Number of elements in the `fields` array
 */
RGL_API rgl_status_t rgl_node_points_format(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies YieldPointsNode.
 * The Node is a marker of what fields are expected by the user.
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param fields Subsequent fields expected to be available
 * @param field_count Number of elements in the `fields` array
 */
RGL_API rgl_status_t rgl_node_points_yield(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies CompactPointsNode.
 * The Node removes non-hit points. In other words, it converts a point cloud into a dense one.
 * Graph input: point cloud
 * Graph output: point cloud (compacted)
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 */
RGL_API [[deprecated("Use rgl_node_points_compact_by_field(rgl_node_t* node, rgl_field_t field) instead.")]] rgl_status_t
rgl_node_points_compact(rgl_node_t* node);

/**
 * Creates or modifies CompactPointsByFieldNode.
 * The Node removes points if the given field is set to a non-zero value.
 * Currently supported fields are RGL_FIELD_IS_HIT_I32 and RGL_FIELD_IS_GROUND_I32.
 * In other words, it converts a point cloud into a dense one.
 * Graph input: point cloud
 * Graph output: point cloud (compacted)
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param field Field by which points will be removed. Has to be RGL_FIELD_IS_HIT_I32 or RGL_FIELD_IS_GROUND_I32.
 */
RGL_API rgl_status_t rgl_node_points_compact_by_field(rgl_node_t* node, rgl_field_t field);

/**
 * Creates or modifies SpatialMergePointsNode.
 * The Node merges point clouds spatially (e.g., multiple lidars outputs into one point cloud).
 * Only provided fields are merged (RGL_FIELD_DYNAMIC_FORMAT is not supported).
 * Input point clouds must be unorganized (height == 1).
 * Any modification to the Node's parameters clears accumulated data.
 * Graph input: point cloud(s)
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param fields Fields to be merged.
 * @param field_count Number of elements in the `fields` array.
 */
RGL_API rgl_status_t rgl_node_points_spatial_merge(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies TemporalMergePointsNode.
 * The Node accumulates (performs temporal merge on) point clouds on each run.
 * Only provided fields are merged (RGL_FIELD_DYNAMIC_FORMAT is not supported).
 * Input point cloud must be unorganized (height == 1).
 * Any modification to the Node's parameters clears accumulated data.
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param fields Fields to be merged.
 * @param field_count Number of elements in the `fields` array.
 */
RGL_API rgl_status_t rgl_node_points_temporal_merge(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies FromArrayPointsNode.
 * The Node provides initial points for its children Nodes.
 * Input: none
 * Output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param points Pointer to the array of points. A point is represented as a structure composed of fields.
 * See RGLFields.hpp (https://github.com/RobotecAI/RobotecGPULidar/blob/main/src/RGLFields.hpp).
 * Example of that structure:
 * struct ExamplePoint
 * {
 *   Field<XYZ_VEC3_F32>::type xyz;
 *   Field<PADDING_32>::type padding;
 *   Field<IS_HIT_I32>::type isHit;
 *   Field<INTENSITY_F32>::type intensity;
 * };
 * @param points_count Number of elements in the `points` array.
 * @param rgl_field_t Subsequent fields to be present in the binary input.
 * @param field_count Number of elements in the `fields` array.
 */
RGL_API rgl_status_t rgl_node_points_from_array(rgl_node_t* node, const void* points, int32_t points_count,
                                                const rgl_field_t* fields, int32_t field_count);

/**
 * Creates or modifies RadarPostprocessPointsNode.
 * The Node processes point cloud to create radar-like output.
 * The point cloud is reduced by clustering input based on hit-point attributes: distance, radial speed and azimuth.
 * Some radar parameters may vary for different distance ranges, as radars may employ multiple frequency bands.
 * For this reason, the configuration allows the definition of multiple radar scopes of parameters on the assumption that:
 *   - in case of scopes that overlap, the first matching one will be used
 *   - if the point is not within any of the radar scopes, it will be rejected
 * The output consists of the collection of one point per cluster (the closest to the azimuth and elevation center).
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param radar_scopes Array of radar scopes of parameters. See `rgl_radar_scope_t` for more details.
 * @param radar_scopes_count Number of elements in the `radar_scopes` array.
 * @param ray_azimuth_step The azimuth step between rays (in radians).
 * @param ray_elevation_step The elevation step between rays (in radians).
 * @param frequency The frequency of the radar (in Hz).
 */
RGL_API rgl_status_t rgl_node_points_radar_postprocess(rgl_node_t* node, const rgl_radar_scope_t* radar_scopes,
                                                       int32_t radar_scopes_count, float ray_azimuth_step,
                                                       float ray_elevation_step, float frequency);

/**
 * Creates or modifies FilterGroundPointsNode.
 * The Node adds RGL_FIELD_IS_GROUND_I32 which indicates the point is on the ground. Points are not removed.
 * Ground points are defined as those located below the sensor with a normal vector pointing upwards at an angle smaller than the threshold.
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param sensor_up_vector Pointer to single Vec3 describing up vector of depended frame.
 * @param ground_angle_threshold The maximum angle between the sensor's ray and the normal vector of the hit point in radians.
 */
RGL_API rgl_status_t rgl_node_points_filter_ground(rgl_node_t* node, const rgl_vec3f* sensor_up_vector,
                                                   float ground_angle_threshold);

/**
 * Creates or modifies GaussianNoiseAngularRaysNode.
 * Applies angular noise to the rays before raycasting.
 * See documentation: https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md#ray-based-angular-noise
 * Graph input: rays
 * Graph output: rays
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param mean Angular noise mean in radians.
 * @param st_dev Angular noise standard deviation in radians.
 * @param axis Axis on which angular noise will be performed.
 */
RGL_API rgl_status_t rgl_node_gaussian_noise_angular_ray(rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis);

/**
 * Creates or modifies GaussianNoiseAngularHitpointNode.
 * Adds angular noise to already computed hitpoints.
 * Note: affects on RGL_FIELD_XYZ_VEC3_F32 and RGL_DISTANCE_F32.
 * Should be used after the raytrace Node.
 * Using this noise after Nodes that modify XYZ (e.g. points_transform, points_downsample) may cause incorrect values in fields other than RGL_FIELD_XYZ_VEC3_F32.
 * See documentation: https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md#hitpoint-based-angular-noise
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param mean Angular noise mean in radians.
 * @param st_dev Angular noise standard deviation in radians.
 * @param axis Axis on which angular noise will be performed.
 */
RGL_API rgl_status_t rgl_node_gaussian_noise_angular_hitpoint(rgl_node_t* node, float mean, float st_dev,
                                                              rgl_axis_t rotation_axis);

/**
 * Creates or modifies GaussianNoiseDistanceNode.
 * Changes the distance between the hitpoint and the lidar's origin.
 * Note: affects on RGL_FIELD_XYZ_VEC3_F32 and RGL_DISTANCE_F32.
 * Should be used after the raytrace Node.
 * Using this noise after Nodes that modify XYZ (e.g. points_transform, points_downsample) may cause incorrect values in fields other than RGL_FIELD_XYZ_VEC3_F32.
 * See documentation: https://github.com/RobotecAI/RobotecGPULidar/blob/main/docs/GaussianNoise.md#distance-noise
 * Graph input: point cloud
 * Graph output: point cloud
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param mean Distance noise mean in meters.
 * @param st_dev_base Distance noise standard deviation base in meters.
 * @param st_dev_rise_per_meter Distance noise standard deviation rise per meter.
 */
RGL_API rgl_status_t rgl_node_gaussian_noise_distance(rgl_node_t* node, float mean, float st_dev_base,
                                                      float st_dev_rise_per_meter);

/******************************** GRAPH ********************************/

/**
 * Starts execution of the RGL graph containing provided Node.
 * This function is asynchronous.
 * @param node Any Node from the graph to execute
 */
RGL_API rgl_status_t rgl_graph_run(rgl_node_t node);

/**
 * Destroys RGL graph (all connected Nodes) containing the provided Node.
 * @param node Any Node from the graph to destroy
 */
RGL_API rgl_status_t rgl_graph_destroy(rgl_node_t node);

/**
 * Obtains the result information of any Node in the graph.
 * The function will fill output parameters that are not null.
 * I.e., The count of the output elements can be queried using a nullptr out_size_of.
 * @param node Node to get output from
 * @param field Field to get output from. Formatted output with FormatNode should be marked as RGL_FIELD_DYNAMIC_FORMAT.
 * @param out_count Returns the number of available elements (e.g., points). It may be null.
 * @param out_size_of Returns byte size of a single element (e.g., point). It may be null.
 */
RGL_API rgl_status_t rgl_graph_get_result_size(rgl_node_t node, rgl_field_t field, int32_t* out_count, int32_t* out_size_of);

/**
 * Obtains the result data of any Node in the graph.
 * If the result is not yet available, this function will block.
 * @param node Node to get output from
 * @param field Field to get output from. Formatted output with FormatNode should be marked as RGL_FIELD_DYNAMIC_FORMAT.
 * @param data Returns binary data, expects a buffer of size (*out_count) * (*out_size_of) from rgl_graph_get_result_size(...) call.
 */
RGL_API rgl_status_t rgl_graph_get_result_data(rgl_node_t node, rgl_field_t field, void* data);

/**
 * Adds child to the parent Node
 * @param parent Node that will be set as the parent of (child)
 * @param child Node that will be set as the child of (parent)
 */
RGL_API rgl_status_t rgl_graph_node_add_child(rgl_node_t parent, rgl_node_t child);

/**
 * Removes child from the parent Node
 * @param parent Node that will be removed as a parent from (child)
 * @param child Node that will be removed as a child from (parent)
 */
RGL_API rgl_status_t rgl_graph_node_remove_child(rgl_node_t parent, rgl_node_t child);

/**
 * Allows to set relative node's priority, which determine their execution order.
 * Nodes will be executed in descending order of their priorities.
 * This is useful when some branch needs to be executed before another.
 * By default, all nodes have set priority to 0.
 * Setting node priority to a value higher than the priority of its parent will also bump parent's priority.
 * However, setting node priority to a value lower than its parent won't propagate to its parent.
 * It is illegal to set node's priority to a value lower than max priority value of its outputs.
 * @param node Node to set its priority.
 * @param priority Signed integer describing priority.
 */
RGL_API rgl_status_t rgl_graph_node_set_priority(rgl_node_t node, int32_t priority);

/**
 * Returns node's priority.
 * @param node Node to query
 * @param out_priority Non-null pointer where priority will be stored.
 */
RGL_API rgl_status_t rgl_graph_node_get_priority(rgl_node_t node, int32_t* out_priority);
