/**
 * RGL API extensions specific to E2E Simulator needs.
 * Some of these calls may be moved to the stable API in the future.
*/
#pragma once

#include <rgl/api/experimental.h>

/**
 * Formats internal to E2E simulator, defined in its source code, subject to change.
 */
typedef enum
{
	RGL_FORMAT_E2E_INVALID = RGL_FORMAT_COUNT,
	RGL_FORMAT_E2E_PCL12,
	RGL_FORMAT_E2E_PCL24,
	RGL_FORMAT_E2E_PCL48,
	RGL_FORMAT_E2E_COUNT,
} rgl_format_e2e_t;

RGL_API rgl_status_t
rgl_lidar_set_ring_indices(rgl_lidar_t lidar, int* ring_ids, int ring_ids_count);

RGL_API rgl_status_t
rgl_lidar_set_gaussian_noise_params(rgl_lidar_t lidar,
                                    rgl_angular_noise_type_t angular_noise_type,
                                    float angular_noise_stddev,
                                    float angular_noise_mean,
                                    float distance_noise_stddev_base,
                                    float distance_noise_stddev_rise_per_meter,
                                    float distance_noise_mean);

// Note: as of now this affects only E2E formats
RGL_API rgl_status_t
rgl_lidar_set_post_raycast_transform(rgl_lidar_t lidar, rgl_mat3x4f* transform);
