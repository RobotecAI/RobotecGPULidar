#include <helpers/testPointCloud.hpp>

#include <api/apiCommon.hpp>

class RadarTrackObjectsNodeTest : public RGLTest
{};

TEST_F(RadarTrackObjectsNodeTest, creating_objects_test)
{
	// Setup objects tracking node
	rgl_node_t trackObjectsNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode));
}
