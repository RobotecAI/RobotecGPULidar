#include <api/apiCommon.hpp>
#include <helpers/testPointCloud.hpp>

#include <random>

class RadarTrackObjectsNodeTest : public RGLTest
{};

template <float MinV, float MaxV>
float getRandomValue()
{
	const static auto seed = std::random_device{}();
	static std::mt19937 generator(seed);
	static std::uniform_real_distribution<float> distribution(MinV, MaxV);

	return distribution(generator);
};

template <rgl_field_t FieldName, float MinV, float MaxV>
void SetFieldValues(TestPointCloud& pointCloud, size_t pointsCount)
{
	std::vector<typename Field<FieldName>::type> fields;
	std::generate_n(std::back_inserter(fields), pointsCount, [&]() { return getRandomValue<MinV, MaxV>(); });
	pointCloud.setFieldValues<FieldName>(fields);
}

TEST_F(RadarTrackObjectsNodeTest, creating_objects_test)
{
	// Setup objects tracking node
	rgl_node_t trackObjectsNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode));

	const size_t pointsCount = 2;
	std::vector<rgl_field_t> pointFields = Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode)->getRequiredFieldList();
	TestPointCloud inPointCloud(pointFields, pointsCount);

	SetFieldValues<DISTANCE_F32, 5.0f, 6.0f>(inPointCloud, pointsCount);
	SetFieldValues<AZIMUTH_F32, -10.0f, 10.0f>(inPointCloud, pointsCount);
	SetFieldValues<ELEVATION_F32, -5.0f, 5.0f>(inPointCloud, pointsCount);
	SetFieldValues<RADIAL_SPEED_F32, -5.0f, 5.0f>(inPointCloud, pointsCount);
	inPointCloud.setFieldValues<XYZ_VEC3_F32>(generateFieldValues(pointsCount, genCoord));
	SetFieldValues<POWER_F32, 90.0f, 130.0f>(inPointCloud, pointsCount);
	SetFieldValues<RCS_F32, 10.0f, 30.0f>(inPointCloud, pointsCount);
	SetFieldValues<NOISE_F32, 80.0f, 100.0f>(inPointCloud, pointsCount);

	const auto usePointsNode = inPointCloud.createUsePointsNode();
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));
	EXPECT_RGL_SUCCESS(rgl_graph_run(trackObjectsNode));
}
