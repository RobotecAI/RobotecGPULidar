#pragma once

#include <vector>
#include <math/Mat3x4f.hpp>

struct PostProcessNodeParams
{
	rgl_radar_scope_t scope;
	int32_t radarScopesCount;
	float azimuthStepRad;
	float elevationStepRad;
	float frequencyHz;
	float powerTransmittedDdm;
	float antennaGainDbi;
	float noiseMean;
	float noiseStdDev;
};

struct TrackObjectNodeParams
{
	float distanceThreshold;
	float azimuthThreshold;
	float elevationThreshold;
	float radialSpeedThreshold;
	float maxMatchingDistance;
	float maxPredictionTimeFrame;
	float movementSensitivity;
};

#if RGL_BUILD_ROS2_EXTENSION
#include <rgl/api/extensions/ros2.h>

static void constructRadarPostProcessObjectTrackingGraph(
    const std::vector<rgl_mat3x4f>& radarRays, const rgl_mat3x4f& radarRayTf, rgl_node_t& postProcessNode,
    const PostProcessNodeParams& postProcessNodeParams, rgl_node_t& trackObjectNode,
    const TrackObjectNodeParams& trackObjectNodeParams, const std::vector<Field<ENTITY_ID_I32>::type>& entityIds,
    const std::vector<rgl_radar_object_class_t>& objectClasses, bool withPublish = false)
{
	rgl_node_t radarRaysNode = nullptr, radarRaysTfNode = nullptr, raytraceNode = nullptr, noiseNode = nullptr,
	           compactNode = nullptr, formatPostProcessNode = nullptr, formatTrackObjectNode = nullptr,
	           ros2PublishPostProcessNode = nullptr, ros2PublishTrackObjectNode = nullptr;

	ASSERT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&radarRaysNode, radarRays.data(), radarRays.size()));
	ASSERT_RGL_SUCCESS(rgl_node_rays_transform(&radarRaysTfNode, &radarRayTf));
	ASSERT_RGL_SUCCESS(rgl_node_raytrace(&raytraceNode, nullptr));
	ASSERT_RGL_SUCCESS(
	    rgl_node_points_compact_by_field(&compactNode, IS_HIT_I32)); // RadarComputeEnergyPointsNode requires dense input
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_postprocess(
	    &postProcessNode, &postProcessNodeParams.scope, postProcessNodeParams.radarScopesCount,
	    postProcessNodeParams.azimuthStepRad, postProcessNodeParams.elevationStepRad, postProcessNodeParams.frequencyHz,
	    postProcessNodeParams.powerTransmittedDdm, postProcessNodeParams.antennaGainDbi, postProcessNodeParams.noiseMean,
	    postProcessNodeParams.noiseStdDev));
	ASSERT_RGL_SUCCESS(rgl_node_points_radar_track_objects(
	    &trackObjectNode, trackObjectNodeParams.distanceThreshold, trackObjectNodeParams.azimuthThreshold,
	    trackObjectNodeParams.elevationThreshold, trackObjectNodeParams.radialSpeedThreshold,
	    trackObjectNodeParams.maxMatchingDistance, trackObjectNodeParams.maxPredictionTimeFrame,
	    trackObjectNodeParams.movementSensitivity));
	ASSERT_RGL_SUCCESS(
	    rgl_node_points_radar_set_classes(trackObjectNode, entityIds.data(), objectClasses.data(), entityIds.size()));

	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(radarRaysNode, radarRaysTfNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(radarRaysTfNode, raytraceNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(raytraceNode, compactNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(compactNode, postProcessNode));
	ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(postProcessNode, trackObjectNode));

	if (withPublish) {
		const std::vector<rgl_field_t> formatFields = {XYZ_VEC3_F32, ENTITY_ID_I32};
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatPostProcessNode, formatFields.data(), formatFields.size()));
		ASSERT_RGL_SUCCESS(rgl_node_points_format(&formatTrackObjectNode, formatFields.data(), formatFields.size()));
		ASSERT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2PublishPostProcessNode, "radar_detection", "world"));
		ASSERT_RGL_SUCCESS(rgl_node_points_ros2_publish(&ros2PublishTrackObjectNode, "radar_track_object", "world"));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(trackObjectNode, formatTrackObjectNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(postProcessNode, formatPostProcessNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatPostProcessNode, ros2PublishPostProcessNode));
		ASSERT_RGL_SUCCESS(rgl_graph_node_add_child(formatTrackObjectNode, ros2PublishTrackObjectNode));
	}
}
#endif

static std::vector<rgl_mat3x4f> genRadarRays(const float minAzimuth, const float maxAzimuth, const float minElevation,
                                             const float maxElevation, const float azimuthStep, const float elevationStep)
{
	std::vector<rgl_mat3x4f> rays;
	for (auto a = minAzimuth; a <= maxAzimuth; a += azimuthStep) {
		for (auto e = minElevation; e <= maxElevation; e += elevationStep) {
			// By default, the rays are directed along the Z-axis
			// So first, we rotate them around the Y-axis to point towards the X-axis (to be RVIZ2 compatible)
			// Then, rotation around Z is azimuth, around Y is elevation
			const auto ray = Mat3x4f::rotationDeg(0, e, a) * Mat3x4f::rotationDeg(0, 90, 0);
			rays.emplace_back(ray.toRGL());

			const auto rayDir = ray * Vec3f{0, 0, 1};
			//printf("rayDir: %.2f %.2f %.2f\n", rayDir.x(), rayDir.y(), rayDir.z());
		}
	}

	return rays;
}