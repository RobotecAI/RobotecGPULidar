#pragma once

#include <rgl/api/experimental.h>









// TODO(prybicki):
// rgl_lidar_set_dimensions
// rgl_lidar_wait_for_pipeline


rgl_field_t[] XYZ = {
	RGL_FIELD_X_F32,
	RGL_FIELD_Y_F32,
	RGL_FIELD_Z_F32
};
rgl_field_t[] INTENSITY = {
	INTENSITY
};
rgl_field_t[] XYZI = {
	RGL_FIELD_X_F32,
	RGL_FIELD_Y_F32,
	RGL_FIELD_Z_F32,
	RGL_FIELD_INTENSITY_F32
};
rgl_field_t[] PCL24 = {
	RGL_FIELD_X_F32,
	RGL_FIELD_Y_F32,
	RGL_FIELD_Z_F32,
	RGL_FIELD_PADDING_32,
	RGL_FIELD_INTENSITY_F32,
	RGL_FIELD_RING_ID_U16
};
rgl_mat3x4f ros_transform = {/* ... */};
rgl_mat3x4f[] vls128_rays = {/* ... */};
#define SIZE(array) (sizeof(array) / sizeof(*array);

void example()
{
rgl_node_t vis_points = nullptr; // Visualize points in Unity (as a point cloud)
rgl_node_t vis_image = nullptr; // Visualize intensity data as a 2D image
rgl_node_t pcd = nullptr; // Write captured data to a file
rgl_node_t pcl24 = nullptr; // Send point cloud over ROS2 topic

// Create root node of the pipeline (the beginning)
rgl_node_t root = nullptr;
rgl_pipeline_create(&root);

//////////////////////////////////////////////////////////////////////
// Dense points saved to a PCD file to create 3D map for e.g. Autoware localization
rgl_pipeline_format(&pcd, root, XYZI, SIZE(XYZI));
rgl_pipeline_downsample(&pcd, pcd, 0.01);
rgl_pipeline_write_pcd_file(&pcd, pcd, "/tmp/output.pcd");

///////////////////////////////////////
// Common path for vis_points and pcl24
rgl_node_t compacted = nullptr;
rgl_pipeline_compact(&compacted, root);

//////////////////////////////////////////
// Dense points for visualization in Unity
rgl_pipeline_format(&vis_points, compacted, XYZ, SIZE(XYZ));

///////////////////////////////////////////////
// Dense points to be published on a ROS2 topic
rgl_pipeline_format(&pcl24, compacted, PCL24, SIZE(PCL24));
rgl_pipeline_transform(&pcl24, pcl24, ros_transform);
rgl_pipeline_publish_ros2_topic(&pcl24, pcl24, "/lidar/vls128");

///////////////////////////////////////////////
// Sparse 2D image for visualization of intensity
rgl_pipeline_format(&vis_image, root, INTENSITY, SIZE(INTENSITY));


// Scene setup omitted for brevity
rgl_scene_t scene = nullptr;

// Setup LiDAR
rgl_lidar_t vls128 = nullptr;
rgl_lidar_create(&vls128, vls128_rays, 204800);
rgl_lidar_set_dimensions(vls128, 1600, 128); // 1600 * 128 == 204800

while (true) {
	rgl_lidar_raytrace_async(scene, vls128); // This will implicitly wait until the previous async request is done.

	// Receive vis_points
	int vis_point_size, vis_hitpoints_count;
	rgl_node_point_get_size(vis_points, &vis_point_size);
	rgl_pipeline_node_get_results_size(vls128, vis_points, &vis_hitpoints_count);
	void* data = malloc(vis_point_size * vis_hitpoints_count);

	// Similarly, vis_image can be obtained, omitted for brevity

	// Channels pcd and pcl48 will send data to their destinations automatically, no action required.

	// Optionally, user can explicit wait for the pipeline to finish,
	// e.g. to make sure data was published before taking further action.
	rgl_lidar_synchronize(vls128);
}

}