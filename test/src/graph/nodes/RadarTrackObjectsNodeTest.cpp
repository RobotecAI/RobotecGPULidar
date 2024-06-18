#include <api/apiCommon.hpp>
#include <helpers/testPointCloud.hpp>

#include <random>


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

void generateDetectionFields(const Vec3f& clusterCenter, const Vec3f& clusterSpread, size_t clusterPointsCount,
                             Field<ENTITY_ID_I32>::type clusterId, std::vector<Vec3f>& xyz, std::vector<float>& distance,
                             std::vector<float>& azimuth, std::vector<float>& elevation, std::vector<float>& radialSpeed,
                             std::vector<Field<ENTITY_ID_I32>::type>& entityIds,
                             std::vector<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type>& absVelocities,
                             std::vector<Field<RELATIVE_VELOCITY_VEC3_F32>::type>& relVelocities)
{
	const auto clusterXYZ = generateFieldValues(clusterPointsCount, genNormal);
	for (const auto& detectionXYZ : clusterXYZ) {
		const auto worldXYZ = detectionXYZ * clusterSpread + clusterCenter;
		const auto worldSph = worldXYZ.toSpherical();

		xyz.emplace_back(worldXYZ);
		distance.emplace_back(worldSph[0]);
		azimuth.emplace_back(worldSph[1]);
		elevation.emplace_back(worldSph[2]);
		radialSpeed.emplace_back(getRandomValue<float, 4.8f, 5.2f>());
		entityIds.emplace_back(clusterId);
		absVelocities.emplace_back(Vec3f{0});
		relVelocities.emplace_back(Vec3f{0});
	}
}

void generateDetectionCluster(const Vec3f& clusterCenter, const Vec3f& clusterSpread, size_t clusterPointsCount,
                              Field<ENTITY_ID_I32>::type clusterId, TestPointCloud& pointCloud)
{
	std::vector<Vec3f> xyz;
	std::vector<float> distance, azimuth, elevation, radialSpeed;
	std::vector<Field<ENTITY_ID_I32>::type> entityIds;
	std::vector<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type> absVelocities;
	std::vector<Field<RELATIVE_VELOCITY_VEC3_F32>::type> relVelocities;
	generateDetectionFields(clusterCenter, clusterSpread, clusterPointsCount, clusterId, xyz, distance, azimuth, elevation,
	                        radialSpeed, entityIds, absVelocities, relVelocities);

	pointCloud.setFieldValues<XYZ_VEC3_F32>(xyz);
	pointCloud.setFieldValues<DISTANCE_F32>(distance);
	pointCloud.setFieldValues<AZIMUTH_F32>(azimuth);
	pointCloud.setFieldValues<ELEVATION_F32>(elevation);
	pointCloud.setFieldValues<RADIAL_SPEED_F32>(radialSpeed);
	pointCloud.setFieldValues<ENTITY_ID_I32>(entityIds);
	pointCloud.setFieldValues<ABSOLUTE_VELOCITY_VEC3_F32>(absVelocities);
	pointCloud.setFieldValues<RELATIVE_VELOCITY_VEC3_F32>(relVelocities);
}

void generateFixedDetectionClusters(TestPointCloud& pointCloud, size_t clusterCount, size_t clusterPointsCount)
{
	constexpr float centerScale = 10.0f;
	const Vec3f clusterSpread = {1.0f};

	std::vector<Vec3f> xyz;
	std::vector<float> distance, azimuth, elevation, radialSpeed;
	std::vector<Field<ENTITY_ID_I32>::type> entityIds;
	std::vector<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type> absVelocities;
	std::vector<Field<RELATIVE_VELOCITY_VEC3_F32>::type> relVelocities;

	for (int i = 0; i < clusterCount; ++i) {
		const auto angle = i * 2 * M_PI / static_cast<double>(clusterCount);
		const auto clusterCenter = Vec3f{std::cos(angle), std::sin(angle), 0.0f} * centerScale;
		generateDetectionFields(clusterCenter, clusterSpread, clusterPointsCount, i, xyz, distance, azimuth, elevation,
		                        radialSpeed, entityIds, absVelocities, relVelocities);
	}

	pointCloud.setFieldValues<XYZ_VEC3_F32>(xyz);
	pointCloud.setFieldValues<DISTANCE_F32>(distance);
	pointCloud.setFieldValues<AZIMUTH_F32>(azimuth);
	pointCloud.setFieldValues<ELEVATION_F32>(elevation);
	pointCloud.setFieldValues<RADIAL_SPEED_F32>(radialSpeed);
	pointCloud.setFieldValues<ENTITY_ID_I32>(entityIds);
	pointCloud.setFieldValues<ABSOLUTE_VELOCITY_VEC3_F32>(absVelocities);
	pointCloud.setFieldValues<RELATIVE_VELOCITY_VEC3_F32>(relVelocities);
}

void generateRandomDetectionClusters(TestPointCloud& pointCloud, size_t clusterCount, size_t clusterPointsCount)
{
	constexpr float centerScale = 10.0f;
	const Vec3f clusterSpread = {1.0f};
	const Vec3f centerOffset = Vec3f{20.0f, 0.0f, 0.0f};

	std::vector<Vec3f> xyz;
	std::vector<float> distance, azimuth, elevation, radialSpeed;
	std::vector<Field<ENTITY_ID_I32>::type> entityIds;
	std::vector<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type> absVelocities;
	std::vector<Field<RELATIVE_VELOCITY_VEC3_F32>::type> relVelocities;

	for (int i = 0; i < clusterCount; ++i) {
		const auto clusterCenter = getRandomVector() * centerScale + centerOffset;
		generateDetectionFields(clusterCenter, clusterSpread, clusterPointsCount, i, xyz, distance, azimuth, elevation,
		                        radialSpeed, entityIds, absVelocities, relVelocities);
	}

	pointCloud.setFieldValues<XYZ_VEC3_F32>(xyz);
	pointCloud.setFieldValues<DISTANCE_F32>(distance);
	pointCloud.setFieldValues<AZIMUTH_F32>(azimuth);
	pointCloud.setFieldValues<ELEVATION_F32>(elevation);
	pointCloud.setFieldValues<RADIAL_SPEED_F32>(radialSpeed);
	pointCloud.setFieldValues<ENTITY_ID_I32>(entityIds);
	pointCloud.setFieldValues<ABSOLUTE_VELOCITY_VEC3_F32>(absVelocities);
	pointCloud.setFieldValues<RELATIVE_VELOCITY_VEC3_F32>(relVelocities);
}

rgl_radar_object_class_t getObjectClass(const RadarTrackObjectsNode::ObjectState& objectState)
{
	if (objectState.classificationProbabilities.classCar > 0) {
		return RGL_RADAR_CLASS_CAR;
	}
	if (objectState.classificationProbabilities.classTruck > 0) {
		return RGL_RADAR_CLASS_TRUCK;
	}
	if (objectState.classificationProbabilities.classMotorcycle > 0) {
		return RGL_RADAR_CLASS_MOTORCYCLE;
	}
	if (objectState.classificationProbabilities.classBicycle > 0) {
		return RGL_RADAR_CLASS_BICYCLE;
	}
	if (objectState.classificationProbabilities.classPedestrian > 0) {
		return RGL_RADAR_CLASS_PEDESTRIAN;
	}
	if (objectState.classificationProbabilities.classAnimal > 0) {
		return RGL_RADAR_CLASS_ANIMAL;
	}
	if (objectState.classificationProbabilities.classHazard > 0) {
		return RGL_RADAR_CLASS_HAZARD;
	}
	return RGL_RADAR_CLASS_UNKNOWN;
}

TEST_F(RadarTrackObjectsNodeTest, objects_number_test)
{
	constexpr float distanceThreshold = 2.0f;
	constexpr float azimuthThreshold = 0.5f;
	constexpr float elevationThreshold = 0.5f;
	constexpr float radialSpeedThreshold = 0.5f;

	constexpr float maxMatchingDistance = 1.0f;
	constexpr float maxPredictionTimeFrame = 500.0f;
	constexpr float movementSensitivity = 0.01;

	rgl_node_t trackObjectsNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode, distanceThreshold, azimuthThreshold,
	                                                       elevationThreshold, radialSpeedThreshold, maxMatchingDistance,
	                                                       maxPredictionTimeFrame, movementSensitivity));

	constexpr size_t objectsCount = 5;
	constexpr size_t detectionsCountPerObject = 10;
	std::vector<rgl_field_t> pointFields = Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode)->getRequiredFieldList();
	TestPointCloud inPointCloud(pointFields, objectsCount * detectionsCountPerObject);
	generateFixedDetectionClusters(inPointCloud, objectsCount, detectionsCountPerObject);

	std::set<Field<ENTITY_ID_I32>::type> uniqueEntityIds;
	for (auto entityId : inPointCloud.getFieldValues<ENTITY_ID_I32>()) {
		uniqueEntityIds.insert(entityId);
	}

	std::vector<Field<ENTITY_ID_I32>::type> entityIds;
	std::vector<rgl_radar_object_class_t> objectClasses;
	for (auto entityId : uniqueEntityIds) {
		// Object class is assigned just in some arbitrary way.
		entityIds.push_back(entityId);
		objectClasses.push_back(static_cast<rgl_radar_object_class_t>(entityId % RGL_RADAR_CLASS_COUNT));
	}
	ASSERT_RGL_SUCCESS(
	    rgl_node_points_radar_set_classes(trackObjectsNode, entityIds.data(), objectClasses.data(), entityIds.size()));

	const auto usePointsNode = inPointCloud.createUsePointsNode();
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));
	ASSERT_RGL_SUCCESS(rgl_graph_run(trackObjectsNode));

	// Verify objects count on the output.
	int32_t detectedObjectsCount = 0, objectsSize = 0;
	ASSERT_RGL_SUCCESS(rgl_graph_get_result_size(trackObjectsNode, XYZ_VEC3_F32, &detectedObjectsCount, &objectsSize));
	ASSERT_EQ(detectedObjectsCount, objectsCount);

	// Verify object classes - matching between entity ids and object classes is hold internally but not exposed through public interface.
	// Object states are not directly connected to entities - they have their own ids. These are reasons behind following checkouts.
	auto trackObjectsNodePtr = Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode);
	const auto& objectStates = trackObjectsNodePtr->getObjectStates();
	std::unordered_map<rgl_radar_object_class_t, int> objectClassDeclaredCounts, objectClassDetectedCounts;
	for (auto objectClass : objectClasses) {
		++objectClassDeclaredCounts[objectClass];
	}
	for (const auto& objectState : objectStates) {
		++objectClassDetectedCounts[getObjectClass(objectState)];
	}
	for (const auto& declaredClassCounts : objectClassDeclaredCounts) {
		const auto it = objectClassDetectedCounts.find(declaredClassCounts.first);
		ASSERT_TRUE(it != objectClassDetectedCounts.cend());
		ASSERT_EQ(it->second, declaredClassCounts.second);
	}
}

TEST_F(RadarTrackObjectsNodeTest, tracking_kinematic_object_test)
{
	constexpr float distanceThreshold = 2.0f;
	constexpr float azimuthThreshold = 0.5f;
	constexpr float elevationThreshold = 0.5f;
	constexpr float radialSpeedThreshold = 0.5f;

	constexpr float maxMatchingDistance = 1.0f;
	constexpr float maxPredictionTimeFrame = 500.0f;
	constexpr float movementSensitivity = 0.01;

	rgl_node_t trackObjectsNode = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode, distanceThreshold, azimuthThreshold,
	                                                       elevationThreshold, radialSpeedThreshold, maxMatchingDistance,
	                                                       maxPredictionTimeFrame, movementSensitivity));

	constexpr size_t detectionsCount = 10;
	const Vec3f clusterSpread = {1.0f};
	const Vec3f& initialCloudTranslation = Vec3f{5.0f, -3.0f, 0.0f};
	const Vec3f iterationTranslation = Vec3f{0.0f, 0.1f, 0.0f};
	const uint64_t frameTimeNs = 5 * 1e6; // ms

	const int numberOfIterations = 60;
	int iterationCounter = 0;
	while (iterationCounter < numberOfIterations) {
		auto trackObjectsNodePtr = Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode);
		TestPointCloud inPointCloud(trackObjectsNodePtr->getRequiredFieldList(), detectionsCount);
		generateDetectionCluster(initialCloudTranslation + static_cast<float>(iterationCounter) * iterationTranslation,
		                         clusterSpread, detectionsCount, 0, inPointCloud);

		auto usePointsNode = inPointCloud.createUsePointsNode();
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));

		ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, iterationCounter * frameTimeNs));
		ASSERT_RGL_SUCCESS(rgl_graph_run(trackObjectsNode));

		ASSERT_RGL_SUCCESS(rgl_graph_node_remove_child(usePointsNode, trackObjectsNode));

		{
			const auto& objectStates = trackObjectsNodePtr->getObjectStates();
			ASSERT_EQ(objectStates.size(),
			          1); // Only one group of detections is generated, and they are assumed to be part of the same object.

			const auto& checkedObjectState = objectStates.front();
			ASSERT_NEAR(checkedObjectState.lastMeasuredTime, 1e-6 * iterationCounter * frameTimeNs, 1e-6);

			if (iterationCounter > 0) {
				ASSERT_EQ(checkedObjectState.objectStatus, RadarTrackObjectsNode::ObjectStatus::Measured);
				ASSERT_EQ(checkedObjectState.movementStatus, RadarTrackObjectsNode::MovementStatus::Moved);
			}
		}

		++iterationCounter;
	}
}

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>
TEST_F(RadarTrackObjectsNodeTest, creating_random_objects_test)
{
	GTEST_SKIP_("Debug test on development stage.");

	std::vector<rgl_field_t> fields{XYZ_VEC3_F32};

	constexpr float distanceThreshold = 2.0f;
	constexpr float azimuthThreshold = 0.1f;
	constexpr float elevationThreshold = 0.1f;
	constexpr float radialSpeedThreshold = 0.5f;

	constexpr float maxMatchingDistance = 1.0f;
	constexpr float maxPredictionTimeFrame = 500.0f;
	constexpr float movementSensitivity = 0.01;

	size_t iterationCounter = 0;
	while (true) {
		// Setup objects tracking node
		rgl_node_t trackObjectsNode = nullptr, ros2DetectionsNode = nullptr, ros2ObjectsNode = nullptr,
		           detectionsFormat = nullptr, objectsFormat = nullptr;
		ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode, distanceThreshold, azimuthThreshold,
		                                                       elevationThreshold, radialSpeedThreshold, maxMatchingDistance,
		                                                       maxPredictionTimeFrame, movementSensitivity));
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

TEST_F(RadarTrackObjectsNodeTest, tracking_objects_test)
{
	GTEST_SKIP_("Debug test on development stage.");

	std::vector<rgl_field_t> detectionFields{XYZ_VEC3_F32};
	std::vector<rgl_field_t> objectFields{XYZ_VEC3_F32, ENTITY_ID_I32};

	constexpr float distanceThreshold = 2.0f;
	constexpr float azimuthThreshold = 0.1f;
	constexpr float elevationThreshold = 0.1f;
	constexpr float radialSpeedThreshold = 0.5f;

	constexpr float maxMatchingDistance = 1.0f;
	constexpr float maxPredictionTimeFrame = 500.0f;
	constexpr float movementSensitivity = 0.01;

	// Setup objects tracking node
	rgl_node_t trackObjectsNode = nullptr, ros2DetectionsNode = nullptr, ros2ObjectsNode = nullptr, detectionsFormat = nullptr,
	           objectsFormat = nullptr;
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(&trackObjectsNode, distanceThreshold, azimuthThreshold,
	                                                       elevationThreshold, radialSpeedThreshold, maxMatchingDistance,
	                                                       maxPredictionTimeFrame, movementSensitivity));
	ASSERT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2DetectionsNode, "radar_detections", "world"));
	ASSERT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2ObjectsNode, "radar_objects", "world"));
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&detectionsFormat, detectionFields.data(), detectionFields.size()));
	ASSERT_RGL_SUCCESS(rgl_node_points_format(&objectsFormat, objectFields.data(), objectFields.size()));

	constexpr size_t objectsCount = 5;
	constexpr size_t detectionsCountPerObject = 10;
	std::vector<rgl_field_t> pointFields = Node::validatePtr<RadarTrackObjectsNode>(trackObjectsNode)->getRequiredFieldList();
	TestPointCloud inPointCloud(pointFields, objectsCount * detectionsCountPerObject);

	generateFixedDetectionClusters(inPointCloud, objectsCount, detectionsCountPerObject);

	auto usePointsNode = inPointCloud.createUsePointsNode();
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, detectionsFormat));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(detectionsFormat, ros2DetectionsNode));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(trackObjectsNode, objectsFormat));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(objectsFormat, ros2ObjectsNode));

	const uint64_t frameTime = 5 * 1e6; // ms
	int iterationCounter = 0;
	while (true) {
		ASSERT_RGL_SUCCESS(rgl_scene_set_time(nullptr, iterationCounter * frameTime));
		ASSERT_RGL_SUCCESS(rgl_graph_run(trackObjectsNode));

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		rgl_graph_node_remove_child(usePointsNode, trackObjectsNode);
		rgl_graph_node_remove_child(usePointsNode, detectionsFormat);

		inPointCloud.transform(Mat3x4f::rotationDeg(0.0f, 0.0f, 5.0f));
		usePointsNode = inPointCloud.createUsePointsNode();

		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, trackObjectsNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, detectionsFormat));

		++iterationCounter;
	}
}
#endif
