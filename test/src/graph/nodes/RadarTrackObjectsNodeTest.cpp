#include <api/apiCommon.hpp>
#include <helpers/testPointCloud.hpp>
#include <rgl/api/extensions/ros2.h>

#include <random>
#include <ranges>


class RadarTrackObjectsNodeTest : public RGLTest
{};

template<typename Type, Type MinV, Type MaxV>
Type getRandomValue()
{
	static_assert(std::is_arithmetic_v<Type>, "Template arguments are not numbers.");
	static std::uniform_real_distribution<float> distribution(MinV, MaxV);

	const auto seed = std::random_device{}();
	std::mt19937 generator(seed);

	return distribution(generator);
}

Vec3f getRandomVector()
{
	std::uniform_real_distribution<> distribution(0, 2 * M_PI);

	const auto seed = std::random_device{}();
	std::mt19937 generator(seed);

	double theta = distribution(generator), phi = distribution(generator);
	return Vec3f{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
}

void generateDetectionClusters(TestPointCloud& pointCloud, const std::vector<Vec3f>& clusterCenters, size_t clusterPointsCount)
{
	std::vector<Vec3f> xyz;
	std::vector<float> distance;
	std::vector<float> azimuth;
	std::vector<float> elevation;
	std::vector<float> radialSpeed;

	for (const auto clusterCenter : clusterCenters) {
		const auto clusterXYZ = generateFieldValues(clusterPointsCount, genNormal);
		for (const auto& detectionXYZ : clusterXYZ) {
			const auto worldXYZ = detectionXYZ + clusterCenter;

			xyz.emplace_back(worldXYZ);
			distance.emplace_back(worldXYZ.length());

			const auto worldSph = worldXYZ.toSpherical();
			azimuth.emplace_back(worldSph[1]);
			elevation.emplace_back(worldSph[2]);

			radialSpeed.emplace_back(getRandomValue<float, 4.8f, 5.2f>());
		}
	}

	pointCloud.setFieldValues<XYZ_VEC3_F32>(xyz);
	pointCloud.setFieldValues<DISTANCE_F32>(distance);
	pointCloud.setFieldValues<AZIMUTH_F32>(azimuth);
	pointCloud.setFieldValues<ELEVATION_F32>(elevation);
	pointCloud.setFieldValues<RADIAL_SPEED_F32>(radialSpeed);
}

void generateFixedDetectionClusters(TestPointCloud& pointCloud, size_t clusterCount, size_t clusterPointsCount)
{
	constexpr float centerScale = 10.0f;
	const Vec3f centerXOffset = Vec3f{20.0f, 0.0f, 0.0f};

	std::vector<Vec3f> clusterCenters;
	for (int i = 0; i < clusterCount; ++i) {
		const auto angle = i * 2 * M_PI / static_cast<double>(clusterCount);
		clusterCenters.emplace_back(Vec3f{std::cos(angle), std::sin(angle), 0.0f} * centerScale);
	}

	generateDetectionClusters(pointCloud, clusterCenters, clusterPointsCount);
}

void generateRandomDetectionClusters(TestPointCloud& pointCloud, size_t clusterCount, size_t clusterPointsCount)
{
	constexpr float centerScale = 10.0f;
	const Vec3f centerOffset = Vec3f{20.0f, 0.0f, 0.0f};

	std::vector<Vec3f> clusterCenters;
	std::generate_n(std::back_inserter(clusterCenters), clusterCount,
	                [&]() { return getRandomVector() * centerScale + centerOffset; });

	generateDetectionClusters(pointCloud, clusterCenters, clusterPointsCount);
}

TEST_F(RadarTrackObjectsNodeTest, objects_number_test)
{
	std::vector<rgl_field_t> fields{XYZ_VEC3_F32};

	// Setup objects tracking node
	rgl_node_t trackObjectsNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode));

	constexpr size_t objectsCount = 5;
	constexpr size_t detectionsCountPerObject = 10;
	std::vector<rgl_field_t> pointFields =
	    Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode)->getRequiredFieldList();
	TestPointCloud inPointCloud(pointFields, objectsCount * detectionsCountPerObject);

	generateFixedDetectionClusters(inPointCloud, objectsCount, detectionsCountPerObject);

	const auto usePointsNode = inPointCloud.createUsePointsNode();
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));
	ASSERT_RGL_SUCCESS(rgl_graph_run(trackObjectsNode));

	int32_t detectedObjectsCount = 0, objectsSize = 0;
	rgl_graph_get_result_size(trackObjectsNode, fields.at(0), &detectedObjectsCount, &objectsSize);
	ASSERT_TRUE(detectedObjectsCount == objectsCount);
}

TEST_F(RadarTrackObjectsNodeTest, creating_random_objects_test)
{
	GTEST_SKIP_("Debug test on development stage.");

	std::vector<rgl_field_t> fields{XYZ_VEC3_F32};
	size_t iterationCounter = 0;

	while (true) {
		// Setup objects tracking node
		rgl_node_t trackObjectsNode = nullptr, ros2DetectionsNode = nullptr, ros2ObjectsNode = nullptr,
		           detectionsFormat = nullptr, objectsFormat = nullptr;
		ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode));
		ASSERT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2DetectionsNode, "radar_detections", "world"));
		ASSERT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2ObjectsNode, "radar_objects", "world"));
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&detectionsFormat, fields.data(), fields.size()));
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&objectsFormat, fields.data(), fields.size()));

		const size_t objectsCount = getRandomValue<int, 5, 10>();
		const size_t detectionsCountPerObject = getRandomValue<int, 10, 20>();
		std::vector<rgl_field_t> pointFields =
		    Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode)->getRequiredFieldList();
		TestPointCloud inPointCloud(pointFields, objectsCount * detectionsCountPerObject);

		generateRandomDetectionClusters(inPointCloud, objectsCount, detectionsCountPerObject);

		const auto usePointsNode = inPointCloud.createUsePointsNode();
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));

		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, detectionsFormat));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(detectionsFormat, ros2DetectionsNode));

		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(trackObjectsNode, objectsFormat));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(objectsFormat, ros2ObjectsNode));

		ASSERT_RGL_SUCCESS(rgl_graph_run(trackObjectsNode));
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		int32_t detectedObjectsCount = 0, objectsSize = 0;
		rgl_graph_get_result_size(trackObjectsNode, fields.at(0), &detectedObjectsCount, &objectsSize);

		if (detectedObjectsCount != objectsCount) {
			printf("[%lu] Detected / given objects: %d / %lu\n", iterationCounter++, detectedObjectsCount, objectsCount);
		}

		EXPECT_RGL_SUCCESS(rgl_cleanup());
	}
}
