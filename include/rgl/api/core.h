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
#include <type_traits>
#endif

#ifdef __cplusplus
#define NO_MANGLING extern "C"
#else // NOT __cplusplus
#define NO_MANGLING
#endif

#if defined RGL_STATIC
#define RGL_VISIBLE
#else
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
#endif // RGL_STATIC

#define RGL_API NO_MANGLING RGL_VISIBLE

#define RGL_VERSION_MAJOR 0
#define RGL_VERSION_MINOR 18
#define RGL_VERSION_PATCH 0

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

#ifdef __cplusplus
static_assert(sizeof(rgl_vec2f) == 2 * sizeof(float));
static_assert(std::is_trivial<rgl_vec2f>::value);
static_assert(std::is_standard_layout<rgl_vec2f>::value);
#endif

/**
 * Three consecutive 32-bit floats.
 */
typedef struct
{
	float value[3];
} rgl_vec3f;

#ifdef __cplusplus
static_assert(sizeof(rgl_vec3f) == 3 * sizeof(float));
static_assert(std::is_trivial<rgl_vec3f>::value);
static_assert(std::is_standard_layout<rgl_vec3f>::value);
#endif

/**
 * Three consecutive 32-bit signed integers.
 */
typedef struct
{
	int32_t value[3];
} rgl_vec3i;

#ifdef __cplusplus
static_assert(sizeof(rgl_vec3i) == 3 * sizeof(int32_t));
static_assert(std::is_trivial<rgl_vec3i>::value);
static_assert(std::is_standard_layout<rgl_vec3i>::value);
#endif

/**
 * Row-major matrix with 3 rows and 4 columns of 32-bit floats.
 * Right-handed coordinate system.
 */
typedef struct
{
	float value[3][4];
} rgl_mat3x4f;

#ifdef __cplusplus
static_assert(sizeof(rgl_mat3x4f) == 3 * 4 * sizeof(float));
static_assert(std::is_trivial<rgl_mat3x4f>::value);
static_assert(std::is_standard_layout<rgl_mat3x4f>::value);
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

#ifdef __cplusplus
static_assert(sizeof(rgl_radar_scope_t) == 5 * sizeof(float));
static_assert(std::is_trivial<rgl_radar_scope_t>::value);
static_assert(std::is_standard_layout<rgl_radar_scope_t>::value);
#endif

/**
 * Describes 4 bone weights affecting a mesh vertex.
 * The sum of all weights for a given vertex should equal 1.
 * If a vertex is affected by fewer than 4 bones, each of the remaining weight values must be 0.
 * bone_idxes for unused bones must still be valid (filled with the existing bone indexes).
 */
typedef struct
{
	/**
	 * Array for skinning weights.
	 * The weight at each index corresponds to the bone_idx with the same index.
	 */
	float weights[4];
	/**
	 * Array for 4 bone indexes affecting a vertex.
	 */
	int32_t bone_idxes[4];
} rgl_bone_weights_t;

#ifdef __cplusplus
static_assert(sizeof(rgl_bone_weights_t) == 4 * sizeof(float) + 4 * sizeof(int32_t));
static_assert(std::is_trivial<rgl_bone_weights_t>::value);
static_assert(std::is_standard_layout<rgl_bone_weights_t>::value);
#endif

/**
 * Radar object class used in object tracking.
 */
typedef enum : int32_t
{
	RGL_RADAR_CLASS_CAR = 0,
	RGL_RADAR_CLASS_TRUCK,
	RGL_RADAR_CLASS_MOTORCYCLE,
	RGL_RADAR_CLASS_BICYCLE,
	RGL_RADAR_CLASS_PEDESTRIAN,
	RGL_RADAR_CLASS_ANIMAL,
	RGL_RADAR_CLASS_HAZARD,
	RGL_RADAR_CLASS_UNKNOWN,
	RGL_RADAR_CLASS_COUNT
} rgl_radar_object_class_t;

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
	RGL_EXTENSION_WEATHER = 3,
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
	/**
	 * Strength of the returned signal captured by the LiDAR sensor.
	 * It is simulated using intensity textures assigned to entities (see `rgl_entity_set_intensity_texture`).
	 * The final value also depends on the incident angle of the ray hit.
	 */
	RGL_FIELD_INTENSITY_F32,
	/**
	 * Same as RGL_FIELD_INTENSITY_F32, but uint8_t type.
	 */
	RGL_FIELD_INTENSITY_U8,
	RGL_FIELD_IS_HIT_I32,
	RGL_FIELD_IS_GROUND_I32,
	RGL_FIELD_RAY_IDX_U32,
	RGL_FIELD_ENTITY_ID_I32,
	RGL_FIELD_DISTANCE_F32,
	/**
	 * Azimuth angle of the hit point in radians.
	 * Currently only compatible with engines that generate rays as follows:
	 * uses a left-handed coordinate system, rotation applies in ZXY order, up vector is Y, forward vector is Z
	 */
	RGL_FIELD_AZIMUTH_F32,
	/**
	 * Elevation angle of the hit point in radians.
	 * Currently only compatible with engines that generate rays as follows:
	 * uses a left-handed coordinate system, rotation applies in ZXY order, up vector is Y, forward vector is Z
	 */
	RGL_FIELD_ELEVATION_F32,
	RGL_FIELD_RING_ID_U16,
	RGL_FIELD_RETURN_TYPE_U8,
	/**
	 * Seconds have passed since the time of the sensor trigger when this point was measured.
	 * If velocity distortion is disabled, the time stamp for all points will be zero.
	 */
	RGL_FIELD_TIME_STAMP_F64,
	/**
	 * Nanoseconds have passed since the time of the sensor trigger when this point was measured.
	 * If velocity distortion is disabled, the time stamp for all points will be zero.
	 */
	RGL_FIELD_TIME_STAMP_U32,

	/**
	 * Velocity of the hit point on the entity.
	 * It depends on entity's
	 * - linear velocity
	 * - angular velocity
	 * - mesh deformations (e.g. skinning)
	 * The aforementioned are inferred from calls to
	 *  `rgl_entity_set_transform`, `rgl_scene_set_time` and `rgl_entity_apply_external_animation`, `rgl_entity_set_pose_world`.
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

	/**
	 * Lidar reflective value. Similar to the `RGL_FIELD_INTENSITY_F32` but set as a single value for the entire entity.
	 * Could be replaced with `RGL_FIELD_INTENSITY_F32` and a 1x1 texture when float-type texture will be supported.
	 * For non-hit points zero is assigned.
	 */
	RGL_FIELD_LASER_RETRO_F32,

	// Dummy fields
	RGL_FIELD_PADDING_8 = 1024,
	RGL_FIELD_PADDING_16,
	RGL_FIELD_PADDING_32,
	// Dynamic fields
	RGL_FIELD_DYNAMIC_FORMAT = 13842,
} rgl_field_t;

/**
 * Kinds of return type for multi-return LiDAR output.
 */
typedef enum : int32_t
{
	RGL_RETURN_TYPE_NOT_DIVERGENT = 0,
	RGL_RETURN_TYPE_FIRST = 1,
	RGL_RETURN_TYPE_LAST = 2,
} rgl_return_type_t;

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
 * @param mesh Mesh to modify
 * @param uvs An array of rgl_vec2f or binary-compatible data representing Mesh uv coordinates
 * @param vertex_count Number of elements in the vertices array. It has to be equal to vertex buffer size.
 */
RGL_API rgl_status_t rgl_mesh_set_texture_coords(rgl_mesh_t mesh, const rgl_vec2f* uvs, int32_t uv_count);

/**
 * Assign bone weights to given Mesh.
 *
 * @param mesh Mesh to modify.
 * @param bone_weights An array of rgl_bone_weights_t objects that associate vertices with the bones affecting them.
                       The bone weights object at each index corresponds to the vertex with the same index (`vertices` array of `rgl_mesh_create` API call).
 * @param bone_weights_count Number of elements in the bone_weights array. It must be equal to the vertex count of the Mesh!
 */
RGL_API rgl_status_t rgl_mesh_set_bone_weights(rgl_mesh_t mesh, const rgl_bone_weights_t* bone_weights,
                                               int32_t bone_weights_count);

/**
 * Assign restposes to given Mesh.
 *
 * @param mesh Mesh to modify.
 * @param restposes An array containing inverse of the transformation matrix of the bone in restpose for each bone.
 *                  Restpose at each index in the array corresponds to the bone with the same index.
 *                  Typically, the restpose is the same as the bindpose.
 * @param bones_count Number of elements in the restposes array.
 */
RGL_API rgl_status_t rgl_mesh_set_restposes(rgl_mesh_t mesh, const rgl_mat3x4f* restposes, int32_t bones_count);

/**
 * Informs that the given Mesh will be no longer used.
 * The Mesh will be destroyed after all referring Entities are destroyed.
 * @param mesh Mesh to be marked as no longer needed
 */
RGL_API rgl_status_t rgl_mesh_destroy(rgl_mesh_t mesh);

/**
 * Assigns value true to out_alive if the given mesh is known and has not been destroyed,
 * assigns value false otherwise.
 * @param mesh Mesh to check if alive
 * @param out_alive Boolean set to indicate if alive
 */
RGL_API rgl_status_t rgl_mesh_is_alive(rgl_mesh_t mesh, bool* out_alive);

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
RGL_API rgl_status_t rgl_entity_set_transform(rgl_entity_t entity, const rgl_mat3x4f* transform);

/**
 * Set the current pose of the given Entity in world coordinates.
 * The pose stands for bone transforms used in skeleton animation.
 * The mesh associated with this entity must have bone weights and restposes assigned.
 * If the pose of an entity is being set, the API call `rgl_entity_set_transform` should no longer be called on this entity.
 * Should be called after rgl_scene_set_time to ensure proper velocity computation.
 * @param entity Entity to modify.
 * @param pose An array containing transformation matrices of the bones in world coordinates.
 *             Bone transform at each index corresponds to the bone with the same index.
 * @param bones_count Number of elements in the pose array. It must be equal to restposes count in the associated mesh!
 */
RGL_API rgl_status_t rgl_entity_set_pose_world(rgl_entity_t entity, const rgl_mat3x4f* pose, int32_t bones_count);

/**
 * Set instance ID of the given Entity.
 * @param entity Entity to modify
 * @param int ID to set. If not set, value of the Entity id will be automatically generated as a DEFAULT_ENTITY_ID.
 */
RGL_API rgl_status_t rgl_entity_set_id(rgl_entity_t entity, int32_t id);

/**
 * Assign intensity texture to the given Entity. The assumption is that the Entity can hold only one intensity texture.
 * @param entity Entity to modify.
 * @param texture Texture to assign.
 */
RGL_API rgl_status_t rgl_entity_set_intensity_texture(rgl_entity_t entity, rgl_texture_t texture);

/**
 * Set laser retro value for the given Entity.
 * The value can be retrieved from `RGL_FIELD_LASER_RETRO_F32` point cloud field.
 * Default retro for the Entity is zero.
 * @param entity Entity to modify.
 * @param retro Laser retro value to set.
 */
RGL_API rgl_status_t rgl_entity_set_laser_retro(rgl_entity_t entity, float retro);

/**
 * Provides updated vertices to the Entity resulted from external animation system.
 * It does not modify Mesh API object bound to the Entity.
 * The number of vertices must not change.
 * Should be called after rgl_scene_set_time to ensure proper velocity computation.
 * @param entity Entity to modify
 * @param vertices An array of rgl_vec3f or binary-compatible data representing Mesh vertices
 * @param vertex_count Number of elements in the vertices array. It must be equal to the original vertex count!
 */
RGL_API rgl_status_t rgl_entity_apply_external_animation(rgl_entity_t entity, const rgl_vec3f* vertices, int32_t vertex_count);

/**
 * Assigns value true to out_alive if the given entity is known and has not been destroyed,
 * assigns value false otherwise.
 * @param entity Entity to check if alive
 * @param out_alive Boolean set to indicate if alive
 */
RGL_API rgl_status_t rgl_entity_is_alive(rgl_entity_t entity, bool* out_alive);

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
 * @param texture Texture to be marked as no longer needed
 */
RGL_API rgl_status_t rgl_texture_destroy(rgl_texture_t texture);

/**
 * Assigns value true to out_alive if the given texture is known and has not been destroyed,
 * assigns value false otherwise.
 * @param texture Texture to check if alive
 * @param out_alive Boolean set to indicate if alive
 */
RGL_API rgl_status_t rgl_texture_is_alive(rgl_texture_t texture, bool* out_alive);

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

/**
 * Modifies RaytraceNode to apply sensor velocity.
 * Necessary for velocity distortion or calculating fields: RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32 and RGL_FIELD_RADIAL_SPEED_F32.
 * Relative velocity calculation:
 * To calculate relative velocity the pipeline must allow to compute absolute velocities. For more details refer to API calls documentation:
 * `rgl_scene_set_time`, `rgl_entity_set_transform`, `rgl_entity_set_pose_world`, and `rgl_entity_apply_external_animation`
 * @param node RaytraceNode to modify
 * @param linear_velocity 3D vector for linear velocity in units per second.
 * @param angular_velocity 3D vector for angular velocity in radians per second (roll, pitch, yaw).
 */
RGL_API rgl_status_t rgl_node_raytrace_configure_velocity(rgl_node_t node, const rgl_vec3f* linear_velocity,
                                                          const rgl_vec3f* angular_velocity);

/**
 * Modifies RaytraceNode to apply sensor distortion.
 * Requires time offsets set to rays using rgl_node_rays_set_time_offsets.
 * NOTE:
 * The distortion takes into account only sensor velocity. The velocity of the objects being scanned by the sensor is not considered.
 * Use rgl_node_raytrace_configure_velocity to set sensor velocity.
 * @param node RaytraceNode to modify
 * @param enable If true, velocity distortion feature will be enabled.
 */
RGL_API rgl_status_t rgl_node_raytrace_configure_distortion(rgl_node_t node, bool enable);

/**
 * Modifies RaytraceNode to set non-hit values for distance.
 * Default non-hit value for the RGL_FIELD_DISTANCE_F32 field is set to infinity.
 * This function allows to set custom values:
 *  - for non-hits closer than a minimum range (`nearDistance`),
 *  - for non-hits beyond a maximum range (`farDistance`).
 * Concurrently, it computes the RGL_FIELD_XYZ_VEC3_F32 field for these non-hit scenarios based on these distances, along with ray origin and direction.
 * @param node RaytraceNode to modify.
 * @param nearDistance Distance value for non-hits closer than minimum range.
 * @param farDistance Distance value for non-hits beyond maximum range.
 */
RGL_API rgl_status_t rgl_node_raytrace_configure_non_hits(rgl_node_t node, float nearDistance, float farDistance);

/**
 * Modifies RaytraceNode to apply mask for non-hit values.
 * Masked rays will be non-hits and will have the default non-hit values no matter of raytracing result.
 * @param node RaytraceNode to modify.
 * @param rays_mask Pointer to the array of int32_t. 1 means point is hit, 0 means point is non-hit.
 * @param rays_count Number of elements in the `points_mask` array.
 */
RGL_API rgl_status_t rgl_node_raytrace_configure_mask(rgl_node_t node, const int8_t* rays_mask, int32_t rays_count);

/**
 * Modifies RaytraceNode to set beam divergence.
 * Beam divergence is used to calculate the beam width at the distance of hit point.
 * Setting beam divergence > 0.0f is required to use query for multi-return results.
 * Setting beam divergence == 0.0f disables multi-return.
 * @param node RaytraceNode to modify.
 * @param horizontal_beam_divergence Horizontal beam divergence in radians.
 * @param vertical_beam_divergence Vertical beam divergence in radians.
 */
RGL_API rgl_status_t rgl_node_raytrace_configure_beam_divergence(rgl_node_t node, float horizontal_beam_divergence,
                                                                 float vertical_beam_divergence);

/**
 * Modifies RaytraceNode to set default intensity.
 * This value will be considered when hitting entities with no intensity texture set (`rgl_entity_set_intensity_texture`)
 * Defaulted default intensity is set to zero.
 * When accessing `RGL_FIELD_INTENSITY_U8` float is cast to uint8_t type with clamping at uint8_t max value (255).
 * @param node RaytraceNode to modify.
 * @param default_intensity Default intensity to set (cannot be a negative number).
 */
RGL_API rgl_status_t rgl_node_raytrace_configure_default_intensity(rgl_node_t node, float default_intensity);

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
 * @param frequency The operating frequency of the radar (in Hz).
 * @param power_transmitted The power transmitted by the radar (in dBm).
 * @param cumulative_device_gain The gain of the radar's antennas and any other gains of the device (in dBi).
 * @param received_noise_mean The mean of the received noise (in dB).
 * @param received_noise_st_dev The standard deviation of the received noise (in dB).
 */
RGL_API rgl_status_t rgl_node_points_radar_postprocess(rgl_node_t* node, const rgl_radar_scope_t* radar_scopes,
                                                       int32_t radar_scopes_count, float ray_azimuth_step,
                                                       float ray_elevation_step, float frequency, float power_transmitted,
                                                       float cumulative_device_gain, float received_noise_mean,
                                                       float received_noise_st_dev);

/**
 * Creates or modifies RadarTrackObjectsNode.
 * The Node takes radar detections point cloud as input (from rgl_node_points_radar_postprocess), groups detections into objects based on given thresholds (node parameters),
 * and calculates various characteristics of these objects. The output from this Node is in the form of a point cloud, where point XYZ coordinates are tracked objects positions.
 * Additionally, cloud points have tracked object IDs. Note that for correct calculation and publishing some of object characteristics (e.g. velocity) user has to call
 * rgl_scene_set_time for current scene.
 * The node expects input in world frame to be able to perform prediction properly.
 * Object's orientation, length, and width calculations assume the forward axis is Z and the left axis is -X.
 * Graph input: point cloud, containing additionally RGL_FIELD_DISTANCE_F32, RGL_FIELD_AZIMUTH_F32, RGL_FIELD_ELEVATION_F32, RGL_FIELD_RADIAL_SPEED_F32 and ENTITY_ID_I32.
 * Graph output: point cloud, contains only XYZ_VEC3_F32 for tracked object positions and ENTITY_ID_I32 for their IDs.
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param object_distance_threshold The maximum distance between a radar detection and other closest detection classified as the same tracked object (in meters).
 * @param object_azimuth_threshold The maximum azimuth difference between a radar detection and other closest detection classified as the same tracked object (in radians).
 * @param object_elevation_threshold The maximum elevation difference between a radar detection and other closest detection classified as the same tracked object (in radians).
 * @param object_radial_speed_threshold The maximum radial speed difference between a radar detection and other closest detection classified as the same tracked object (in meters per second).
 * @param max_matching_distance The maximum distance between predicted (based on previous frame) and newly detected object position to match objects between frames (in meters).
 * @param max_prediction_time_frame The maximum time that object state can be predicted until it will be declared lost unless measured again (in milliseconds).
 * @param movement_sensitivity The maximum velocity for an object to be qualified as stationary (in meters per seconds).
 */
RGL_API rgl_status_t rgl_node_points_radar_track_objects(rgl_node_t* node, float object_distance_threshold,
                                                         float object_azimuth_threshold, float object_elevation_threshold,
                                                         float object_radial_speed_threshold, float max_matching_distance,
                                                         float max_prediction_time_frame, float movement_sensitivity);

/**
 * Modifies RadarTrackObjectsNode to set entity ids to radar object classes mapping.
 * This is necessary to call for RadarTrackObjectsNode to classify tracked objects correctly. If not set, objects will be classified as RGL_RADAR_CLASS_UNKNOWN by default.
 * Note that entity can only belong to a single class - passing entity id multiple times in entity_ids array will result in the last version to be assigned. There is no
 * limit on how many entities can have the same class.
 * @param node RadarTrackObjectsNode to modify.
 * @param entity_ids Array of RGL entity ids.
 * @param object_classes Array of radar object classes.
 * @param count Number of elements in entity_ids and object_classes arrays.
 */
RGL_API rgl_status_t rgl_node_points_radar_set_classes(rgl_node_t node, const int32_t* entity_ids,
                                                       const rgl_radar_object_class_t* object_classes, int32_t count);

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

/**
 * Creates or modifies MultiReturnSwitchNode
 * This is a special node which does not modify the data but acts as an adapter to the multi-return feature.
 * Thanks to this node, user can attach unchanged pipelines to work with specific return type from multi-return raytracing.
 * Graph input: point cloud (with multi-return fields)
 * Graph output: point cloud (with a selected field from parent's multi-return point cloud)
 * @param node If (*node) == nullptr, a new Node will be created. Otherwise, (*node) will be modified.
 * @param return_type Return type to select from multi-return point cloud.
 */
RGL_API rgl_status_t rgl_node_multi_return_switch(rgl_node_t* node, rgl_return_type_t);

/**
 * Assigns value true to out_alive if the given node is known and has not been destroyed,
 * assigns value false otherwise.
 * @param node Node to check if alive
 * @param out_alive Boolean set to indicate if alive
 */
RGL_API rgl_status_t rgl_node_is_alive(rgl_node_t node, bool* out_alive);

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
