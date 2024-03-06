#include <helpers/testPointCloud.hpp>
#include <helpers/commonHelpers.hpp>
#include <helpers/graphHelpers.hpp>

#include <api/apiCommon.hpp>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>
#include <ranges>

class RadarPostprocessPointsNodeTest : public RGLTest
{};

TEST_F(RadarPostprocessPointsNodeTest, clustering_test)
{
	// Setup radar node
	rgl_node_t radarNode = nullptr;
	std::array<rgl_radar_scope_t, 2> radarScopes = {
	    rgl_radar_scope_t{ .begin_distance = 0.0f,
	                      .end_distance = 10.0f,
	                      .distance_separation_threshold = 2.0f,
	                      .radial_speed_separation_threshold = 2.0f,
	                      .azimuth_separation_threshold = 2.0f},
	    rgl_radar_scope_t{.begin_distance = 10.0f,
	                      .end_distance = 20.0f,
	                      .distance_separation_threshold = 1.0f,
	                      .radial_speed_separation_threshold = 1.0f,
	                      .azimuth_separation_threshold = 1.0f},
	};
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_postprocess(&radarNode, radarScopes.data(), radarScopes.size(), 1.0f, 1.0f, 1.0f));

	auto runAndReturnNumberOfClusters = [&radarNode](const TestPointCloud& inPointCloud) -> int32_t {
		rgl_node_t usePointsNode = inPointCloud.createUsePointsNode();
		EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, radarNode));
		EXPECT_RGL_SUCCESS(rgl_graph_run(radarNode));
		int32_t outCount = -1;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(radarNode, DISTANCE_F32, &outCount, nullptr));
		EXPECT_RGL_SUCCESS(rgl_graph_node_remove_child(usePointsNode, radarNode));
		EXPECT_RGL_SUCCESS(rgl_graph_destroy(usePointsNode));
		return outCount;
	};

	// We need to provide all required fields, but we will only modify those on which clustering is based
	std::vector<rgl_field_t> pointFields = Node::validatePtr<RadarPostprocessPointsNode>(radarNode)->getRequiredFieldList();

	// TODO(msz-rai): add randomization
	{
		TestPointCloud inPointCloud(pointFields, 2);
		inPointCloud.setFieldValues<DISTANCE_F32>({100.0f, 200.0f});
		inPointCloud.setFieldValues<AZIMUTH_F32>({10.0f, 10.0f});
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({10.0f, 10.0f});
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 0) << "Should reject all points (out of radar scope)";
	}
	{
		TestPointCloud inPointCloud(pointFields, 3);
		inPointCloud.setFieldValues<DISTANCE_F32>({5.0f, 6.0f, 7.0f});
		inPointCloud.setFieldValues<AZIMUTH_F32>({10.0f, 10.1f, 10.2f});
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({10.0f, 10.1f, 10.2f});
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 1)
		    << "Creating clusters phase: Should merge points into one cluster (they all meet separation thresholds)";
	}
	{
		TestPointCloud inPointCloud(pointFields, 2);
		inPointCloud.setFieldValues<DISTANCE_F32>({5.0f, 10.0f});
		inPointCloud.setFieldValues<AZIMUTH_F32>({10.0f, 10.0f});
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({10.0f, 10.0f});
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 2)
		    << "Creating clusters phase: Should not merge because of distance";
	}
	{
		TestPointCloud inPointCloud(pointFields, 2);
		inPointCloud.setFieldValues<DISTANCE_F32>({5.0f, 5.0f});
		inPointCloud.setFieldValues<AZIMUTH_F32>({10.0f, 20.0f});
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({10.0f, 10.0f});
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 2)
		    << "Creating clusters phase: Should not merge because of azimuth";
	}
	{
		TestPointCloud inPointCloud(pointFields, 2);
		inPointCloud.setFieldValues<DISTANCE_F32>({5.0f, 5.0f});
		inPointCloud.setFieldValues<AZIMUTH_F32>({10.0f, 10.0f});
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({10.0f, 20.0f});
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 2)
		    << "Creating clusters phase: Should not merge because of radial speed";
	}
	{
		TestPointCloud inPointCloud(pointFields, 3);
		inPointCloud.setFieldValues<DISTANCE_F32>({0.0f, 2.5f, 1.0f});   // point[1] is separate cluster after first iteration
		inPointCloud.setFieldValues<AZIMUTH_F32>({11.0f, 12.5f, 12.0f}); // azimuth[1] is within separation threshold
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({11.0f, 12.5f, 12.0f}); // speed[1] is within separation threshold
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 1)
		    << "Merging clusters phase: Should merge all clusters (attributes within separation threshold)";
	}
	{
		TestPointCloud inPointCloud(pointFields, 3);
		inPointCloud.setFieldValues<DISTANCE_F32>({0.0f, 2.5f, 1.0f});   // point[1] is separate cluster after first iteration
		inPointCloud.setFieldValues<AZIMUTH_F32>({11.0f, 11.5f, 12.0f}); // azimuth[1] is inside cluster azimuth range
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({11.0f, 11.5f, 12.0f}); // speed[1] is inside cluster speed range
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 1)
		    << "Merging clusters phase: Should merge all clusters (attributes inside cluster range)";
	}
	{
		TestPointCloud inPointCloud(pointFields, 3);
		inPointCloud.setFieldValues<DISTANCE_F32>({0.0f, 2.5f, 1.0f});  // point[1] is separate cluster after first iteration
		inPointCloud.setFieldValues<AZIMUTH_F32>({11.0f, 0.0f, 12.0f}); // azimuth[1] out of separation threshold
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({11.0f, 11.5f, 12.0f}); // speed[1] is within separation threshold
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 2)
		    << "Merging clusters phase: Should not merge one cluster because of azimuth";
	}
	{
		TestPointCloud inPointCloud(pointFields, 3);
		inPointCloud.setFieldValues<DISTANCE_F32>({0.0f, 2.5f, 1.0f});   // point[1] is separate cluster after first iteration
		inPointCloud.setFieldValues<AZIMUTH_F32>({11.0f, 11.5f, 12.0f}); // azimuth[1] is within separation threshold
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({11.0f, 0.0f, 12.0f}); // speed[1] out of separation threshold
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 2)
		    << "Merging clusters phase: Should not merge one cluster because of radial speed";
	}
	{
		TestPointCloud inPointCloud(pointFields, 4);
		inPointCloud.setFieldValues<DISTANCE_F32>({6.5f, 8.0f, 9.5f, 11.0f}); // last point is next scope with threshold=1
		inPointCloud.setFieldValues<AZIMUTH_F32>({10.0f, 10.0f, 10.0f, 10.0f});
		inPointCloud.setFieldValues<RADIAL_SPEED_F32>({10.0f, 10.0f, 10.0f, 10.0f});
		ASSERT_EQ(runAndReturnNumberOfClusters(inPointCloud), 2)
		    << "Creating clusters phase: Should not merge all points because different scope is reached with smaller "
		       "separation threshold";
	}
}
