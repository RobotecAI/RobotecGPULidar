#include <gtest/gtest.h>
#include <utils.hpp>
#include <scenes.hpp>
#include <lidars.hpp>
#include <RGLFields.hpp>

#include <math/Mat3x4f.hpp>

class Graph : public RGLAutoSetUp
{
	std::string getFilename() override { return FILENAME; }
	rgl_log_level_t getLogLevel() override { return RGL_LOG_LEVEL_ALL; }
};

#ifdef RGL_BUILD_PCL_EXTENSION
#include <rgl/api/extensions/pcl.h>

TEST_F(Graph, FullLinear)
{
	setupBoxesAlongAxes(nullptr);

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, shear=nullptr, compact=nullptr, downsample=nullptr;

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({5, 5, 5}, {45, 45, 45}).toRGL();
	rgl_mat3x4f shearTf = Mat3x4f::shear({0,0}, {-1, -1}, {0, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&shear, &shearTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_downsample(&downsample, 0.1f, 0.1f, 0.1f));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, shear));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(shear, downsample));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(downsample, "minimal.pcd"));
}

TEST_F(Graph, NodeRemoval)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, transformPts=nullptr, compact=nullptr, downsample=nullptr;
	rgl_node_t temporalMerge=nullptr;
	std::vector<rgl_field_t> tMergeFields = { RGL_FIELD_XYZ_F32 };

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateXTf = Mat3x4f::TRS({3, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, temporalMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Remove compact<->transformPts connection
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateXTf));
	EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Restore compact<->transformPts connection
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateYTf));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Output pointcloud should contain two boxes
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(temporalMerge, "two_boxes_removal.pcd"));
}

TEST_F(Graph, SpatialMerge)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, compact=nullptr;
	rgl_node_t transformPtsZero=nullptr, transformPtsY=nullptr;
	rgl_node_t spatialMerge=nullptr;
	std::vector<rgl_field_t> sMergeFields = { RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32 };

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPtsZero, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPtsY, &translateYTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_spatial_merge(&spatialMerge, sMergeFields.data(), sMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPtsZero));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPtsY));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPtsZero, spatialMerge));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPtsY, spatialMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(spatialMerge, "two_boxes_spatial_merge.pcd"));
}

TEST_F(Graph, TemporalMerge)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, compact=nullptr, transformPts=nullptr;
	rgl_node_t temporalMerge=nullptr;
	std::vector<rgl_field_t> tMergeFields = { RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32 };

	std::vector<rgl_mat3x4f> rays = makeLidar3dRays(360, 180, 0.36, 0.18);
	rgl_mat3x4f lidarPoseTf = Mat3x4f::TRS({0, 0, -5}).toRGL();
	rgl_mat3x4f zeroTf = Mat3x4f::TRS({0, 0, 0}).toRGL();
	rgl_mat3x4f translateYTf = Mat3x4f::TRS({0, 3, 0}).toRGL();

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact));
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &zeroTf));
	EXPECT_RGL_SUCCESS(rgl_node_points_temporal_merge(&temporalMerge, tMergeFields.data(), tMergeFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, compact));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(compact, transformPts));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(transformPts, temporalMerge));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	// Change transform for the next raytrace
	EXPECT_RGL_SUCCESS(rgl_node_points_transform(&transformPts, &translateYTf));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_write_pcd_file(temporalMerge, "two_boxes_temporal_merge.pcd"));
}
#endif

TEST_F(Graph, FormatNodeResults)
{
	auto mesh = makeCubeMesh();

	auto entity = makeEntity(mesh);
	rgl_mat3x4f entityPoseTf = Mat3x4f::identity().toRGL();
	ASSERT_RGL_SUCCESS(rgl_entity_set_pose(entity, &entityPoseTf));

	rgl_node_t useRays=nullptr, raytrace=nullptr, lidarPose=nullptr, format=nullptr;

	std::vector<rgl_mat3x4f> rays = {
		Mat3x4f::TRS({0, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.1, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.2, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.3, 0, 0}).toRGL(),
		Mat3x4f::TRS({0.4, 0, 0}).toRGL()
	};
	rgl_mat3x4f lidarPoseTf = Mat3x4f::identity().toRGL();
	std::vector<rgl_field_t> formatFields = {
		XYZ_F32,
		PADDING_32
	};

	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_node_rays_transform(&lidarPose, &lidarPoseTf));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
	EXPECT_RGL_SUCCESS(rgl_node_points_format(&format, formatFields.data(), formatFields.size()));

	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, lidarPose));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(lidarPose, raytrace));
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(raytrace, format));

	EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

	int32_t outCount, outSizeOf;
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(format, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSizeOf));

	struct FormatStruct
	{
		Field<XYZ_F32>::type xyz;
		Field<PADDING_32>::type padding;
	} formatStruct;

	EXPECT_EQ(outCount, rays.size());
	EXPECT_EQ(outSizeOf, sizeof(formatStruct));

	std::vector<FormatStruct> formatData{(size_t)outCount};
	EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(format, RGL_FIELD_DYNAMIC_FORMAT, formatData.data()));

	for (int i = 0; i < formatData.size(); ++i) {
		EXPECT_NEAR(formatData[i].xyz[0], rays[i].value[0][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[1], rays[i].value[1][3], 1e-6);
		EXPECT_NEAR(formatData[i].xyz[2], 1, 1e-6);
	}
}

/* TEST_F(Pipeline, AWSIM)
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

	rgl_node_t useRays=nullptr, yield=nullptr, raytrace=nullptr, pose=nullptr, ros=nullptr, compact=nullptr, fmt24=nullptr, fmt48=nullptr;


	EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, nullptr, rays.data(), rays.size()));
	EXPECT_RGL_SUCCESS(rgl_pipline_transform_rays(&pose, useRays, &lidarPose));
	EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, pose, nullptr, 1000.0f));
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compact, raytrace));
	// EXPECT_RGL_SUCCESS(rgl_node_points_transform())


	// use_rays -> raytrace -> compact
		// yield (XYZ_F32)
		// transform (ROS)
			// format PCL24
			// format PCL48

} */
