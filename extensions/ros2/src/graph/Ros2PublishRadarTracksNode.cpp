// Copyright 2025 Robotec.AI
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

#include <graph/NodesRos2.hpp>
#include <scene/Scene.hpp>

void Ros2PublishRadarTracksNode::setParameters(const char* topicName, const char* messageFrameId,
                                               rgl_qos_policy_reliability_t qosReliability,
                                               rgl_qos_policy_durability_t qosDurability, rgl_qos_policy_history_t qosHistory,
                                               int32_t qosHistoryDepth)
{
	frameId = messageFrameId;
	auto qos = rclcpp::QoS(qosHistoryDepth);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(qosReliability));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(qosDurability));
	qos.history(static_cast<rmw_qos_history_policy_t>(qosHistory));
	messagePublisher = std::make_unique<Ros2MessagePublisher<MessageT>>(ros2InitGuard->getNode(), topicName, qos);
}

void Ros2PublishRadarTracksNode::ros2ValidateImpl()
{
	Ros2Node::validateImpl();
	input = getExactlyOneInputOfType<RadarTrackObjectsNode>(); // Make sure RadarTrackObjectsNode is on the input

	if (input->getHeight() != 1) {
		throw InvalidPipeline("ROS2 radar track publish supports unorganized pointclouds only");
	}
}

void Ros2PublishRadarTracksNode::ros2EnqueueExecImpl()
{
	auto& ros2Message = messagePublisher->getMessage();

	ros2Message.header.frame_id = frameId;
	ros2Message.header.stamp = Scene::instance().getTime().has_value() ?
	                               Scene::instance().getTime().value().asRos2Msg() :
	                               static_cast<builtin_interfaces::msg::Time>(ros2InitGuard->getNode().get_clock()->now());

	const auto& objectStates = std::dynamic_pointer_cast<RadarTrackObjectsNode>(input)->getObjectStates();
	ros2Message.tracks.resize(objectStates.size());

	int i = 0;
	for (const auto& objectState : objectStates) {
		auto& radarTrack = ros2Message.tracks[i];

		assert(sizeof(objectState.id) == 4); // Be sure that uuid matches object ID.
		radarTrack.uuid.uuid[0] = static_cast<uint8_t>(objectState.id & 0xff);
		radarTrack.uuid.uuid[1] = static_cast<uint8_t>((objectState.id >> 8) & 0xff);
		radarTrack.uuid.uuid[2] = static_cast<uint8_t>((objectState.id >> 16) & 0xff);
		radarTrack.uuid.uuid[3] = static_cast<uint8_t>((objectState.id >> 24) & 0xff);

		constexpr int signalUnfilled = 255; // According to documentation.
		radarTrack.position = ProcessObjectStat<decltype(radarTrack.position)>(objectState.position);
		radarTrack.position = ProcessReferencePoint(radarTrack.position, objectState.orientation.getLastSample(),
		                                            objectState.length.getMean(), objectState.width.getMean(), signalUnfilled);
		radarTrack.velocity = ProcessObjectStat<decltype(radarTrack.velocity)>(objectState.absVelocity);
		radarTrack.acceleration = ProcessObjectStat<decltype(radarTrack.acceleration)>(objectState.absAccel);
		radarTrack.size.set__x(objectState.length.getMean());
		radarTrack.size.set__y(objectState.width.getMean());
		radarTrack.size.set__z(1.0f);

		radarTrack.classification = ProcessObjectProbabilities(objectState.classificationProbabilities);

		radarTrack.position_covariance = ProcessObjectStatCov(objectState.position);
		radarTrack.velocity_covariance = ProcessObjectStatCov(objectState.relVelocity);
		radarTrack.acceleration_covariance = ProcessObjectStatCov(objectState.relAccel);

		// Height (Z coordinate) is not available in objectState - related covariances are set to 0.
		radarTrack.size_covariance[0] = objectState.length.getVariance();
		radarTrack.size_covariance[1] = objectState.length.getStdDev() * objectState.width.getStdDev();
		radarTrack.size_covariance[2] = 0.0f;
		radarTrack.size_covariance[3] = objectState.width.getVariance();
		radarTrack.size_covariance[4] = 0.0f;
		radarTrack.size_covariance[5] = 0.0f;

		++i;
	}

	messagePublisher->publish();
}

#if RGL_BUILD_AGNOCAST_EXTENSION
void Ros2PublishRadarTracksNode::configureAgnocastImpl(bool enable)
{
	bool isAlreadyAgnocast = dynamic_cast<AgnocastMessagePublisher<MessageT>*>(messagePublisher.get()) != nullptr;

	if (enable == isAlreadyAgnocast) {
		return; // The current configuration is correct
	}

	if (enable) {
		messagePublisher = std::make_unique<AgnocastMessagePublisher<MessageT>>(
		    ros2InitGuard->getNode(), messagePublisher->getTopicName(), messagePublisher->getQos());
		return;
	}
	// else
	messagePublisher = std::make_unique<Ros2MessagePublisher<MessageT>>(
	    ros2InitGuard->getNode(), messagePublisher->getTopicName(), messagePublisher->getQos());
}
#endif

geometry_msgs::msg::Point Ros2PublishRadarTracksNode::ProcessReferencePoint(const geometry_msgs::msg::Point& referencePoint,
                                                                            float yaw, float length, float width,
                                                                            int referenceIndex) const
{
	constexpr int referencePointsCount = 9;
	constexpr std::array<std::array<float, 2>, referencePointsCount> referenceToCenter = {
	    {{{-1.0f, -1.0f}},
	     {{-1.0f, 0.0f}},
	     {{-1.0f, 1.0f}},
	     {{0.0f, 1.0f}},
	     {{1.0f, 1.0f}},
	     {{1.0f, 0.0f}},
	     {{1.0f, -1.0f}},
	     {{0.0f, -1.0f}},
	     {{0.0f, 0.0f}}}
    };

	const auto halfLength = 0.5f * length;
	const auto halfWidth = 0.5f * width;
	referenceIndex = std::clamp(referenceIndex, 0, referencePointsCount - 1);

	geometry_msgs::msg::Point center;
	center.set__x(referencePoint.x + std::cos(yaw) * halfLength * referenceToCenter[referenceIndex][0] -
	              std::sin(yaw) * halfWidth * referenceToCenter[referenceIndex][1]);
	center.y = referencePoint.y + std::sin(yaw) * halfLength * referenceToCenter[referenceIndex][0] +
	           std::cos(yaw) * halfWidth * referenceToCenter[referenceIndex][1];
	center.z = referencePoint.z;

	return center;
}

radar_msgs::msg::RadarTrack::_classification_type Ros2PublishRadarTracksNode::ProcessObjectProbabilities(
    const RadarTrackObjectsNode::ClassificationProbabilities& probabilities) const
{
	constexpr int16_t unknownID = 32000;
	constexpr int16_t carID = 32001;
	constexpr int16_t truckID = 32002;
	constexpr int16_t motorcycleID = 32005;
	constexpr int16_t bicycleID = 32006;
	constexpr int16_t pedestrianID = 32007;

	auto maxScore = probabilities.classUnknown;
	radar_msgs::msg::RadarTrack::_classification_type outputClass = unknownID;

	if (probabilities.classCar > maxScore) {
		maxScore = probabilities.classCar;
		outputClass = carID;
	}
	if (probabilities.classTruck > maxScore) {
		maxScore = probabilities.classTruck;
		outputClass = truckID;
	}
	if (probabilities.classMotorcycle > maxScore) {
		maxScore = probabilities.classMotorcycle;
		outputClass = motorcycleID;
	}
	if (probabilities.classBicycle > maxScore) {
		maxScore = probabilities.classBicycle;
		outputClass = bicycleID;
	}
	if (probabilities.classPedestrian > maxScore) {
		maxScore = probabilities.classPedestrian;
		outputClass = pedestrianID;
	}

	return outputClass;
}

std::array<float, 6> Ros2PublishRadarTracksNode::ProcessObjectStatCov(const RunningStats<Vec3f>& objectStat) const
{
	const auto& statVariance = objectStat.getVariance();
	std::array<float, 6> trackStatCov{};
	trackStatCov[0] = statVariance.x();
	trackStatCov[1] = objectStat.getCovarianceXY();
	trackStatCov[2] = objectStat.getCovarianceZX();
	trackStatCov[3] = statVariance.y();
	trackStatCov[4] = objectStat.getCovarianceYZ();
	trackStatCov[5] = statVariance.z();
	return trackStatCov;
}
