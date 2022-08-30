
#include <vector>
#include <rgl/api/experimental.h>
#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <scenes.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	setupBoxesAlongAxes(nullptr);
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

	rgl_node_t rays=nullptr, yield=nullptr, raytrace=nullptr, pose=nullptr, ros=nullptr, compact=nullptr, fmt24=nullptr, fmt48=nullptr;
	// Mat3x4f lidarPose = Mat3x4f::translation()

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()) {

	}

}


