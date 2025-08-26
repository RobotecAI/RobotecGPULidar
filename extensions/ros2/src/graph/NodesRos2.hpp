// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rgl/api/extensions/ros2.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>

#include <graph/Node.hpp>
#include <graph/NodesCore.hpp>
#include <graph/Interfaces.hpp>

#include <Ros2InitGuard.hpp>
#include <MessagePublisher.hpp>

struct Ros2Node : IPointsNodeSingleInput
{
	Ros2Node() { ros2InitGuard = Ros2InitGuard::acquire(); }

	void enqueueExecImpl() final
	{
		if (!rclcpp::ok()) {
			throw InvalidPipeline("Unable to execute Ros2Node because ROS2 has been shut down.");
		}
		ros2EnqueueExecImpl();
	}

	void validateImpl() final
	{
		IPointsNodeSingleInput::validateImpl();
		ros2ValidateImpl();
	}

	/**
	 * Configures usage of Agnocast middleware.
	 */
	void configureAgnocast(bool enable)
	{
#if RGL_BUILD_AGNOCAST_EXTENSION
		if (enable) {
			// Verify that the Agnocast heaphook library is preloaded before initialization.
			// This library must be loaded via LD_PRELOAD for Agnocast's shared memory functionality to work correctly.
			// While Agnocast internally performs this same validation, it calls std::exit on failure,
			// so we check here to provide a more graceful error.
			const char* preloadEnv = std::getenv("LD_PRELOAD");
			const std::string preloads = preloadEnv ? std::string(preloadEnv) : std::string();
			if (preloads.find("libagnocast_heaphook.so") == std::string::npos) {
				throw std::invalid_argument(
				    "Unable to configure Agnocast because libagnocast_heaphook.so is not found in LD_PRELOAD.");
			}
		}
		configureAgnocastImpl(enable);
#else
		throw std::invalid_argument("Unable to configure Agnocast because the library was not built with Agnocast extension.");
#endif
	}

	virtual ~Ros2Node() = default;

protected:
	std::shared_ptr<Ros2InitGuard> ros2InitGuard;

	virtual void ros2EnqueueExecImpl() = 0;
	virtual void ros2ValidateImpl() = 0;
	virtual void configureAgnocastImpl([[maybe_unused]] bool enable)
	{
		throw std::invalid_argument("Unable to configure Agnocast because requested RGL node does not support it.");
	};
};

struct Ros2PublishPointsNode : Ros2Node
{
	using Ptr = std::shared_ptr<Ros2PublishPointsNode>;

	void setParameters(const char* topicName, const char* messageFrameId,
	                   rgl_qos_policy_reliability_t qosReliability = QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_durability_t qosDurability = QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
	                   rgl_qos_policy_history_t qosHistory = QOS_POLICY_HISTORY_SYSTEM_DEFAULT, int32_t qosHistoryDepth = 10);

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

#if RGL_BUILD_AGNOCAST_EXTENSION
	void configureAgnocastImpl(bool enable) override;
#endif

	~Ros2PublishPointsNode() override = default;

private:
	using MessageT = sensor_msgs::msg::PointCloud2;

	static void updateRos2MessageFields(MessageT& ros2Message, const std::vector<rgl_field_t>& fields);

	DeviceAsyncArray<char>::Ptr inputFmtData = DeviceAsyncArray<char>::create(arrayMgr);

	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;
	std::string frameId{};
};


struct Ros2PublishPointVelocityMarkersNode : Ros2Node
{
	using Ptr = std::shared_ptr<Ros2PublishPointVelocityMarkersNode>;

	void setParameters(const char* topicName, const char* frameId, rgl_field_t velocityField);
	std::vector<rgl_field_t> getRequiredFieldList() const override { return {XYZ_VEC3_F32, velocityField}; }

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

#if RGL_BUILD_AGNOCAST_EXTENSION
	void configureAgnocastImpl(bool enable) override;
#endif

	~Ros2PublishPointVelocityMarkersNode() override = default;

private:
	using MessageT = visualization_msgs::msg::Marker;

	std::string frameId;
	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;

	HostPinnedArray<Vec3f>::Ptr pos = HostPinnedArray<Vec3f>::create();
	HostPinnedArray<Vec3f>::Ptr vel = HostPinnedArray<Vec3f>::create();
	rgl_field_t velocityField;
};

struct Ros2PublishRadarScanNode : Ros2Node
{
	void setParameters(const char* topicName, const char* messageFrameId, rgl_qos_policy_reliability_t qosReliability,
	                   rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory, int32_t qosHistoryDepth);
	std::vector<rgl_field_t> getRequiredFieldList() const override
	{
		return {DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32, /* placeholder for amplitude */ PADDING_32};
	}

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

#if RGL_BUILD_AGNOCAST_EXTENSION
	void configureAgnocastImpl(bool enable) override;
#endif

private:
	using MessageT = radar_msgs::msg::RadarScan;

	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;
	std::string frameId{};

	DeviceAsyncArray<char>::Ptr formattedData = DeviceAsyncArray<char>::create(arrayMgr);
	GPUFieldDescBuilder fieldDescBuilder;
};

struct Ros2PublishRadarTracksNode : Ros2Node
{
	void setParameters(const char* topicName, const char* messageFrameId, rgl_qos_policy_reliability_t qosReliability,
	                   rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory, int32_t qosHistoryDepth,
	                   const Mat3x4f& changeOfBasisTf_);

	// Ros2Node
	void ros2ValidateImpl() override;
	void ros2EnqueueExecImpl() override;

#if RGL_BUILD_AGNOCAST_EXTENSION
	void configureAgnocastImpl(bool enable) override;
#endif

private:
	using MessageT = radar_msgs::msg::RadarTracks;

	std::unique_ptr<MessagePublisher<MessageT>> messagePublisher;
	std::string frameId{};

	Mat3x4f changeOfBasisTf;

	geometry_msgs::msg::Point ProcessReferencePoint(const geometry_msgs::msg::Point& referencePoint, float yaw, float length,
	                                                float width, int referenceIndex) const;

	radar_msgs::msg::RadarTrack::_classification_type ProcessObjectProbabilities(
	    const RadarTrackObjectsNode::ClassificationProbabilities& probabilities) const;

	template<typename TrackVecT>
	TrackVecT ProcessObjectStat(const RunningStats<Vec3f>& objectStat) const
	{
		const auto& statRef = changeOfBasisTf.rotation() * objectStat.getLastSample();
		TrackVecT trackStat{};
		trackStat.set__x(statRef.x());
		trackStat.set__y(statRef.y());
		trackStat.set__z(statRef.z());
		return trackStat;
	}

	std::array<float, 6> ProcessObjectStatCov(const RunningStats<Vec3f>& objectStat) const;
};
