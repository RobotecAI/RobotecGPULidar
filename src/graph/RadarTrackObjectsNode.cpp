// Copyright 2024 Robotec.AI
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

#include <graph/NodesCore.hpp>
#include <scene/Scene.hpp>


// TODO(Pawel): Consider adding more output fields here, maybe usable for ROS 2 message or visualization. Consider also returning detections, when object states are returned through public method.
RadarTrackObjectsNode::RadarTrackObjectsNode()
{
	fieldData.emplace(XYZ_VEC3_F32, createArray<HostPinnedArray>(XYZ_VEC3_F32));
	fieldData.emplace(ENTITY_ID_I32, createArray<HostPinnedArray>(ENTITY_ID_I32));
}

void RadarTrackObjectsNode::setParameters(float distanceThreshold, float azimuthThreshold, float elevationThreshold,
                                          float radialSpeedThreshold)
{
	this->distanceThreshold = distanceThreshold;
	this->azimuthThreshold = azimuthThreshold;
	this->elevationThreshold = elevationThreshold;
	this->radialSpeedThreshold = radialSpeedThreshold;
}

void RadarTrackObjectsNode::enqueueExecImpl()
{
	xyzHostPtr->copyFrom(input->getFieldData(XYZ_VEC3_F32));
	distanceHostPtr->copyFrom(input->getFieldData(DISTANCE_F32));
	azimuthHostPtr->copyFrom(input->getFieldData(AZIMUTH_F32));
	elevationHostPtr->copyFrom(input->getFieldData(ELEVATION_F32));
	radialSpeedHostPtr->copyFrom(input->getFieldData(RADIAL_SPEED_F32));

	// Top level in this list is for objects. Bottom level is for detections that belong to specific objects. Below is an initialization of a helper
	// structure for region growing, which starts with a while-loop.
	std::list<std::list<int>> objectIndices;
	for (int i = 0; i < input->getPointCount(); ++i) {
		objectIndices.emplace_back(1, i);
	}

	auto leftIndex = 0;
	while (leftIndex < objectIndices.size() - 1) {
		auto ownIt = objectIndices.begin();
		std::advance(ownIt, leftIndex);
		auto checkedIt = std::next(ownIt);
		bool reorganizationTookPlace = false;

		while (checkedIt != objectIndices.end()) {
			auto& checkedIndices = *checkedIt;

			// TODO(Pawel): In theory, I do not have to check the same indices multiple times when growing this subset in next iterations.
			for (auto ownDetectionId : *ownIt) {
				const auto isPartOfSameObject = [&, distance = distanceHostPtr->at(ownDetectionId),
				                                 azimuth = azimuthHostPtr->at(ownDetectionId),
				                                 elevation = elevationHostPtr->at(ownDetectionId),
				                                 radialSpeed = radialSpeedHostPtr->at(ownDetectionId)](int checkedDetectionId) {
					return std::abs(distanceHostPtr->at(checkedDetectionId) - distance) <= distanceThreshold &&
					       std::abs(azimuthHostPtr->at(checkedDetectionId) - azimuth) <= azimuthThreshold &&
					       std::abs(elevationHostPtr->at(checkedDetectionId) - elevation) <= elevationThreshold &&
					       std::abs(radialSpeedHostPtr->at(checkedDetectionId) - radialSpeed) <= radialSpeedThreshold;
				};

				if (std::any_of(checkedIndices.cbegin(), checkedIndices.cend(), isPartOfSameObject)) {
					ownIt->splice(ownIt->cend(), checkedIndices);
					break;
				}
			}
			if (checkedIndices.empty()) {
				checkedIt = objectIndices.erase(checkedIt);
				reorganizationTookPlace = true;
			} else {
				++checkedIt;
			}
		}

		if (!reorganizationTookPlace) {
			// We are done with growing this object, no more matching detections are available.
			++leftIndex;
		}
	}

	// Calculate positions of objects detected in current frame.
	std::list<Vec3f> newObjectPositions;
	for (const auto& separateObjectIndices : objectIndices) {
		auto& objectCenter = newObjectPositions.emplace_back(0.0f, 0.0f, 0.0f);
		for (const auto detectionIndex : separateObjectIndices) {
			objectCenter += xyzHostPtr->at(detectionIndex);
		}
		objectCenter *= 1 / static_cast<float>(separateObjectIndices.size());
	}

	const auto currentTime = static_cast<uint32_t>(Scene::instance().getTime().value_or(Time::zero()).asMilliseconds());
	const auto deltaTime = currentTime -
	                       static_cast<uint32_t>(Scene::instance().getPrevTime().value_or(Time::zero()).asMilliseconds());

	// Check object list from previous frame and try to find matches with newly detected objects.
	// Later, for newly detected objects without match, create new object state.
	for (auto objectStateIt = objectStates.begin(); objectStateIt != objectStates.end();) {
		auto& objectState = *objectStateIt;
		const auto predictedPosition = PredictObjectPosition(objectState, deltaTime);
		const auto closestObjectPositionIt = std::min_element(
		    newObjectPositions.cbegin(), newObjectPositions.cend(), [&](const Vec3f& a, const Vec3f& b) {
			    return (a - predictedPosition).lengthSquared() < (b - predictedPosition).lengthSquared();
		    });

		// There is a newly detected object (current frame) that matches the predicted position of one of objects from previous frame.
		// Update object from previous frame to newly detected object position and remove this positions for next checkouts.
		if (const auto& closestObjectPosition = *closestObjectPositionIt;
		    (predictedPosition - closestObjectPosition).length() < predictionSensitivity) {
			UpdateObjectState(objectState, closestObjectPosition, ObjectStatus::Measured, currentTime, deltaTime);
			newObjectPositions.erase(closestObjectPositionIt);
			++objectStateIt;
			continue;
		}

		// There is no match for objectState in newly detected object positions. If that object was already detected in previous frame
		// (not predicted, so objectStatus was as in the condition below), then update it based on prediction (changing its objectStatus
		// to ObjectStatus::Predicted).
		if (objectState.objectStatus == ObjectStatus::New || objectState.objectStatus == ObjectStatus::Measured) {
			UpdateObjectState(objectState, predictedPosition, ObjectStatus::Predicted, currentTime, deltaTime);
			++objectStateIt;
			continue;
		}

		// objectState do not have a match in newly detected object positions and it was also on ObjectStatus::Predicted last frame -
		// not this object is considered lost and is removed from object list. Also remove its ID to poll.
		objectIDPoll.push(objectState.id);
		objectStateIt = objectStates.erase(objectStateIt);
	}

	// All newly detected object position that do not have a match in previous frame - create new object state.
	for (const auto& newObjectPosition : newObjectPositions) {
		CreateObjectState(newObjectPosition, currentTime);
	}

	//	printf("Object states count: %lu (%u)\n", objectStates.size(), currentTime);
	//	for (const auto& objectState : objectStates) {
	//		printf("\t%u: (%.2f, %.2f, %.2f)\n", objectState.id, objectState.position.getLastSample().x(),
	//		       objectState.position.getLastSample().y(), objectState.position.getLastSample().z());
	//	}
	UpdateOutputData();
}

std::vector<rgl_field_t> RadarTrackObjectsNode::getRequiredFieldList() const
{
	return {XYZ_VEC3_F32, DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32};
}

Vec3f RadarTrackObjectsNode::PredictObjectPosition(const ObjectState& objectState, uint32_t deltaTimeMs) const
{
	assert(objectState.position.getSameplesCount() > 0);
	const auto deltaTimeSec = 1e-3f * static_cast<float>(deltaTimeMs);
	const auto assumedVelocity = objectState.absAccel.getSameplesCount() > 0 ?
	                                 (objectState.absVelocity.getLastSample() +
	                                  deltaTimeSec * objectState.absAccel.getLastSample()) :
	                                 objectState.absVelocity.getLastSample();
	const auto predictedMovement = deltaTimeSec * assumedVelocity;
	return objectState.position.getLastSample() + Vec3f{predictedMovement.x(), predictedMovement.y(), 0.0f};
}

void RadarTrackObjectsNode::CreateObjectState(const Vec3f& position, uint32_t currentTimeMs)
{
	auto& objectState = objectStates.emplace_back();
	if (objectIDPoll.empty()) {
		// Overflow is technically acceptable below, but for > 4,294,967,295 objects it may cause having repeated IDs.
		objectState.id = objectIDCounter++;
	} else {
		objectState.id = objectIDPoll.front();
		objectIDPoll.pop();
	}

	objectState.creationTime = currentTimeMs;
	objectState.lastUpdateTime = currentTimeMs;
	objectState.objectStatus = ObjectStatus::New;

	// TODO(Pawel): Consider object radial speed (from detections) as a way to decide here.
	objectState.movementStatus = MovementStatus::Invalid; // No good way to determine it.

	// I do not add velocity or acceleration 0.0f samples because this would affect mean and std dev calculation. However, the
	// number of samples between their stats and position will not match.
	objectState.position.addSample(position);

	// TODO(Pawel): This will have to be determined together with detection bounding boxes calculation.
	objectState.orientation.addSample(0.0f);
	objectState.length.addSample(0.0f);
	objectState.width.addSample(0.0f);
}

void RadarTrackObjectsNode::UpdateObjectState(ObjectState& objectState, const Vec3f& newPosition, ObjectStatus objectStatus,
                                              uint32_t currentTimeMs, uint32_t deltaTimeMs)
{
	const auto displacement = newPosition - objectState.position.getLastSample();
	const auto deltaTimeSecInv = 1e3f / static_cast<float>(deltaTimeMs);
	const auto absVelocity = Vec2f{displacement.x() * deltaTimeSecInv, displacement.y() * deltaTimeSecInv};

	// TODO(Pawel): Note that if this will be second frame when this object exists (first detected last frame, now updated), then
	// this will work incorrectly - velocity from previous frame will be 0.0f (not possible to determine for newly created objects).
	// There may be a need to add some frame counting for this (lifetime of objects).
	const auto absAccel = Vec2f{absVelocity.x() - objectState.absVelocity.getLastSample().x(),
	                            absVelocity.y() - objectState.absVelocity.getLastSample().y()};

	objectState.lastUpdateTime = currentTimeMs;
	objectState.objectStatus = objectStatus;
	objectState.movementStatus = displacement.length() > movementSensitivity ? MovementStatus::Moved :
	                                                                           MovementStatus::Stationary;
	objectState.position.addSample(newPosition);

	objectState.orientation.addSample(0.0f); // velocity direction (vector normalized?)?
	objectState.absVelocity.addSample(absVelocity);
	// objectState.relVelocity.addSample(absVelocity - selfVelocity);
	objectState.absAccel.addSample(absAccel);
	// objectState.relAccel.addSample(absAccel - selfAccel);
	objectState.orientationRate.addSample(0.0f); // change in orientation
	objectState.length.addSample(0.0f);          // from bbox
	objectState.width.addSample(0.0f);           // from bbox
}

void RadarTrackObjectsNode::UpdateOutputData()
{
	fieldData[XYZ_VEC3_F32]->resize(objectStates.size(), true, false);
	auto* xyzPtr = static_cast<Vec3f*>(fieldData[XYZ_VEC3_F32]->getRawWritePtr());

	fieldData[ENTITY_ID_I32]->resize(objectStates.size(), true, false);
	auto* idPtr = static_cast<int32_t*>(fieldData[ENTITY_ID_I32]->getRawWritePtr());

	int objectIndex = 0;
	for (const auto& objectState : objectStates) {
		xyzPtr[objectIndex] = objectState.position.getLastSample();
		idPtr[objectIndex] = objectState.id;
		++objectIndex;
	}
}
