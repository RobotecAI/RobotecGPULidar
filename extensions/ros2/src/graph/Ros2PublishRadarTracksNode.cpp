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
		radarTrack.uuid.set__uuid({});

		{
			const auto& objectPosition = objectState.position.getLastSample();
			radarTrack.position.set__x(objectPosition.x());
			radarTrack.position.set__y(objectPosition.y());
			radarTrack.position.set__z(objectPosition.z());
		}
		{
			const auto& objectVelocity = objectState.relVelocity.getLastSample();
			radarTrack.velocity.set__x(objectVelocity.x());
			radarTrack.velocity.set__y(objectVelocity.y());
			radarTrack.velocity.set__z(objectVelocity.z());
		}
		{
			const auto objectAcceleration = objectState.relAccel.getLastSample();
			radarTrack.acceleration.set__x(objectAcceleration.x());
			radarTrack.acceleration.set__y(objectAcceleration.y());
			radarTrack.acceleration.set__z(objectAcceleration.z());
		}
		{
			const Vec3f size = {};
			radarTrack.size.set__x(size.x());
			radarTrack.size.set__y(size.y());
			radarTrack.size.set__z(size.z());
		}

		radarTrack.classification = radar_msgs::msg::RadarTrack::
		    NO_CLASSIFICATION; // TODO(Pawel): Decide if we do not want to classify it based on e.g. velocity (STATIC or DYNAMIC object)

		//radarTrack.position_covariance[0] = objectState.position.getCovarianceXX();
		radarTrack.position_covariance[1] = objectState.position.getCovarianceXY();
		radarTrack.position_covariance[2] = objectState.position.getCovarianceZX();
		//radarTrack.position_covariance[3] = objectState.position.getCovarianceYY();
		radarTrack.position_covariance[4] = objectState.position.getCovarianceYZ();
		//radarTrack.position_covariance[5] = objectState.position.getCovarianceZZ();

		//radarTrack.velocity_covariance[0] = objectState.relVelocity.getCovarianceXX();
		radarTrack.velocity_covariance[1] = objectState.relVelocity.getCovarianceXY();
		radarTrack.velocity_covariance[2] = objectState.relVelocity.getCovarianceZX();
		//radarTrack.velocity_covariance[3] = objectState.relVelocity.getCovarianceYY();
		radarTrack.velocity_covariance[4] = objectState.relVelocity.getCovarianceYZ();
		//radarTrack.velocity_covariance[5] = objectState.relVelocity.getCovarianceZZ();

		//radarTrack.acceleration_covariance[0] = objectState.relAccel.getCovarianceXX();
		radarTrack.acceleration_covariance[1] = objectState.relAccel.getCovarianceXY();
		radarTrack.acceleration_covariance[2] = objectState.relAccel.getCovarianceZX();
		//radarTrack.acceleration_covariance[3] = objectState.relAccel.getCovarianceYY();
		radarTrack.acceleration_covariance[4] = objectState.relAccel.getCovarianceYZ();
		//radarTrack.acceleration_covariance[5] = objectState.relAccel.getCovarianceZZ();

		//radarTrack.size_covariance[0] = objectState.dimensions.getCovarianceXX();
		//radarTrack.size_covariance[1] = objectState.dimensions.getCovarianceXY();
		//radarTrack.size_covariance[2] = objectState.dimensions.getCovarianceZX();
		//radarTrack.size_covariance[3] = objectState.dimensions.getCovarianceYY();
		//radarTrack.size_covariance[4] = objectState.dimensions.getCovarianceYZ();
		//radarTrack.size_covariance[5] = objectState.dimensions.getCovarianceZZ();

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
