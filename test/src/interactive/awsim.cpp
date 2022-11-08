
#include <vector>
#include <rgl/api/core.h>
#include "../../../src/RGLFields.hpp"
#include <math/Mat3x4f.hpp>
#include <scenes.hpp>
#include <macroCheckRGL.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	setupBoxesAlongAxes(nullptr);
	std::vector<rgl_field_t> visFields = {
		XYZ_F32
	};
	std::vector<rgl_field_t> pcl24Fields =  {
		XYZ_F32,
		PADDING_32,
		INTENSITY_F32,
		RING_ID_U16
	};
	std::vector<rgl_field_t> pcl48Fields = {
		XYZ_F32,
		PADDING_32,
		INTENSITY_F32,
		RING_ID_U16,
		AZIMUTH_F32,
		DISTANCE_F32,
		RETURN_TYPE_U8,
		TIME_STAMP_F64
	};

	rgl_node_t raysNode=nullptr, yield=nullptr, raytrace=nullptr, poseNode=nullptr, ros=nullptr, compact=nullptr, fmt24=nullptr, fmt48=nullptr;
	std::vector<rgl_mat3x4f> rays;
	rgl_mat3x4f lidarPose = Mat3x4f::translation(0, 0, 0).toRGL();

	CHECK_RGL(rgl_node_rays_from_mat3x4f(&raysNode, rays.data(), rays.size()));
	CHECK_RGL(rgl_node_rays_transform(&poseNode, &lidarPose));
	CHECK_RGL(rgl_node_raytrace(&raytrace, nullptr, 1000.0f));
	CHECK_RGL(rgl_node_points_compact(&compact));
		CHECK_RGL(rgl_node_points_yield(&yield, visFields.data(), visFields.size()));
		CHECK_RGL(rgl_node_points_format(&fmt24, pcl24Fields.data(), pcl24Fields.size()));
		CHECK_RGL(rgl_node_points_format(&fmt48, pcl48Fields.data(), pcl48Fields.size()));

	CHECK_RGL(rgl_graph_node_add_child(raysNode, poseNode));
	CHECK_RGL(rgl_graph_node_add_child(poseNode, raytrace));
	CHECK_RGL(rgl_graph_node_add_child(raytrace, compact));
		CHECK_RGL(rgl_graph_node_add_child(compact, yield));
		CHECK_RGL(rgl_graph_node_add_child(compact, fmt24));
		CHECK_RGL(rgl_graph_node_add_child(compact, fmt48));

	CHECK_RGL(rgl_graph_run(raysNode));

	// size_t hitCount = 0;
	// std::vector<rgl_vec3f> visXYZ { rays.size() };
	// CHECK_RGL(rgl_graph_get_result(yield, RGL_FIELD_XYZ_F32, &hitCount, nullptr, visXYZ.data()));
	// CHECK_RGL(rgl_graph_get_result(fmt24, fmt24Token,  ))


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()) {
		// CHECK_RGL(rgl_pipeline_transform_rays(&poseNode, raysNode, &lidarPose));
	}

}

