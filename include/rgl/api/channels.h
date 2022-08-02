#pragma once

#include <rgl/api/experimental.h>

/**
 * Context:
 *
 *
 */

typedef enum
{
	RGL_POINT_PRESET_XYZ,
	RGL_POINT_PRESET_XYZI,
	RGL_POINT_PRESET_AWSIM24,
	RGL_POINT_PRESET_AWSIM48,
	RGL_POINT_PRESET_COUNT,
} rgl_point_preset_t;

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

/**
 * Channel objects describe operations to perform on the point cloud after capturing, e.g.:
 * - Cut (format) out requested point features (X, Y, Z, I, etc.) and construct contiguous binary output.
 * - Make point cloud dense (remove non-hits) (perform stream compaction)
 * - Apply filtering (downsampling) to reduce the number of points
 * - Send it automatically somewhere (e.g. PCD file, ROS2 topic, predetermined memory buffer)
 */
typedef void* rgl_channel_t;

////////////////////
// REMOVED API CALLS

// RGL_API rgl_status_t
// rgl_lidar_get_output_data(rgl_lidar_t lidar, rgl_format_t format, void *out_data);

// RGL_API rgl_status_t
// rgl_lidar_get_output_data(rgl_lidar_t lidar, rgl_format_t format, void *out_data);

////////////////
// NEW API CALLS

// Mark the lidar pointcloud as organized with given w/h. This affects metadata only.
RGL_API rgl_status_t
rgl_lidar_set_dimensions(rgl_lidar_t lidar, int width, int height);

// Wait for all channels to finish.
RGL_API rgl_status_t
rgl_lidar_synchronize(rgl_lidar_t);

RGL_API rgl_status_t
rgl_channel_create_from_fields(rgl_channel_t* out_channel, rgl_field_t* fields, int field_count);

RGL_API rgl_status_t
rgl_channel_create_from_preset(rgl_channel_t, rgl_point_preset_t preset);

RGL_API rgl_status_t
rgl_channel_clone(rgl_channel_t* out_channel, rgl_channel_t src_channel);

RGL_API rgl_status_t
rgl_channel_destroy(rgl_channel_t channel);

RGL_API rgl_status_t
rgl_channel_point_get_size(rgl_channel_t channel, int* out_point_byte_size);

// If true, non-hit points will be removed.
RGL_API rgl_status_t
rgl_channel_pointcloud_set_dense(rgl_channel_t channel, bool dense);

// Sets a transform applied to all points to change the coordinate frame.
RGL_API rgl_status_t
rgl_channel_set_post_transform(rgl_channel_t channel, rgl_mat3x4f* tf);

// Reduces the number of points. This is done *after* post-transform.
RGL_API rgl_status_t
rgl_channel_set_downsampling(rgl_channel_t channel, float leaf_size);

// Setups automatic writing to a PCD file.
RGL_API rgl_status_t
rgl_channel_set_sink_pcd_file(rgl_channel_t channel, const char* file_path);

// Setups automatic publishing to a ROS2 topic.
RGL_API rgl_status_t
rgl_channel_set_sink_ros2_topic(rgl_channel_t channel, const char* topic /* QoS settings, etc */);

// Requests lidar to produce output according to the specification of the channel
RGL_API rgl_status_t
rgl_lidar_attach_channel(rgl_lidar_t lidar, rgl_channel_t channel);

// Returns minimal size of the buffer required to receive data from the given channel of the lidar.
RGL_API rgl_status_t
rgl_lidar_channel_get_results_size(rgl_lidar_t lidar, rgl_channel_t channel, int* point_count);

RGL_API rgl_status_t
rgl_lidar_channel_get_results_data(rgl_lidar_t lidar, rgl_channel_t channel, void* data);

void example()
{
	// The following example how a (very) hypothetical scenario could be handled by RGL API
	// We assume, that a demanding user wants to do many things at once:
	rgl_channel_t vis_points = nullptr; // Visualize points in Unity (as a point cloud)
	rgl_channel_t vis_image = nullptr; // Visualize intensity data as a 2D image
	rgl_channel_t pcd = nullptr; // Write captured data to a file
	rgl_channel_t pcl48 = nullptr; // Send point cloud over ROS2 topic

	//////////////////////////////////////////////
	// Visualization points (as mesh in the world)
	rgl_channel_create(&vis_points);
	// Point format can be defined manually
	rgl_channel_point_append_field(vis_points, RGL_FIELD_X_F32);
	rgl_channel_point_append_field(vis_points, RGL_FIELD_Y_F32);
	rgl_channel_point_append_field(vis_points, RGL_FIELD_Z_F32);
	rgl_channel_pointcloud_set_dense(vis_points, true); // Visualization point cloud should not contain non hit points.


	////////////////////////////////////////
	// Visualization as a 2D intensity image
	rgl_channel_create(&vis_image);
	rgl_channel_point_append_field(vis_image, RGL_FIELD_INTENSITY_F32);
	rgl_channel_pointcloud_set_dense(vis_image, false);


	//////////////////////////////////////////////////////////////////////
	// Saved to a PCD file to create 3D map for e.g. Autoware localization
	rgl_channel_create(&pcd);
	rgl_channel_point_from_preset(pcd, RGL_POINT_PRESET_XYZI);
	rgl_channel_pointcloud_set_dense(pcd, true);
	rgl_channel_set_downsampling(pcd, 0.1);
	rgl_channel_set_sink_pcd_file(pcd, "/tmp/output.pcd");


	////////////////////////////
	// Published on a ROS2 topic
	rgl_channel_create(&pcl48);
	rgl_channel_point_from_preset(pcl48, RGL_POINT_PRESET_AWSIM48);
	rgl_channel_pointcloud_set_dense(pcl48, true);
	rgl_channel_set_post_transform(pcl48, ros_tf);
	rgl_channel_set_sink_ros2_topic(pcl48, "/lidar/vls128");

	rgl_lidar_t vls128 = nullptr;
	rgl_lidar_create(&vls128, vls128_rays, 204800);
	rgl_lidar_set_dimensions(vls128, 1600, 128); // 1600 * 128 == 204800
	rgl_lidar_attach_channel(vls128, vis_points); // Same effect with: rgl_lidar_attach_channel(vls128, nullptr) - default channel
	rgl_lidar_attach_channel(vls128, vis_image);
	rgl_lidar_attach_channel(vls128, pcd);
	rgl_lidar_attach_channel(vls128, pcl48);

	// Scene setup omitted for brevity
	rgl_scene_t scene = nullptr;

	while (true) {
		rgl_lidar_raytrace_async(scene, vls128); // This will implicitly wait until the previous async request is done.

		// Receive vis_points
		int vis_point_size, vis_hitpoints_count;
		rgl_channel_point_get_size(vis_points, &vis_point_size);
		rgl_lidar_channel_get_results_size(vls128, vis_points, &vis_hitpoints_count);
		void* data = malloc(vis_point_size * vis_hitpoints_count);

		// Similarly, vis_image can be obtained, omitted for brevity

		// Channels pcd and pcl48 will send data to their destinations automatically, no action required.

		// Optionally, user can explicit wait for all channels to finish,
		// e.g. to make sure data was published before taking further action.
		rgl_lidar_synchronize(vls128);
	}
}

