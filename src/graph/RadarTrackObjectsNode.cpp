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
                                          float radialSpeedThreshold, float maxMatchingDistance, float maxPredictionTimeFrame,
                                          float movementSensitivity)
{
	this->distanceThreshold = distanceThreshold;
	this->azimuthThreshold = azimuthThreshold;
	this->elevationThreshold = elevationThreshold;
	this->radialSpeedThreshold = radialSpeedThreshold;

	this->maxMatchingDistance = maxMatchingDistance;
	this->maxPredictionTimeFrame = maxPredictionTimeFrame;
	this->movementSensitivity = movementSensitivity;
}

void RadarTrackObjectsNode::setObjectClasses(const int32_t* entityIds, const rgl_radar_object_class_t* objectClasses,
                                             int32_t count)
{
	entityIdsToClasses.clear();
	for (int i = 0; i < count; ++i) {
		entityIdsToClasses[static_cast<Field<ENTITY_ID_I32>::type>(entityIds[i])] = objectClasses[i];
	}
}

void RadarTrackObjectsNode::enqueueExecImpl()
{
	xyzHostPtr->copyFrom(input->getFieldData(XYZ_VEC3_F32));
	distanceHostPtr->copyFrom(input->getFieldData(DISTANCE_F32));
	azimuthHostPtr->copyFrom(input->getFieldData(AZIMUTH_F32));
	elevationHostPtr->copyFrom(input->getFieldData(ELEVATION_F32));
	radialSpeedHostPtr->copyFrom(input->getFieldData(RADIAL_SPEED_F32));
	entityIdHostPtr->copyFrom(input->getFieldData(ENTITY_ID_I32));

	// TODO(Pawel): Reconsider approach below.
	// At this moment, I would like to check input this way, because it will keep RadarTrackObjectsNode testable without
	// an input being RadarPostprocessPointsNode. If I have nullptr here, I simply do not process bounding boxes for detections.
	auto radarPostprocessPointsNode = std::dynamic_pointer_cast<RadarPostprocessPointsNode>(input);
	const auto detectionAabbs = radarPostprocessPointsNode ? radarPostprocessPointsNode->getClusterAabbs() :
	                                                         std::vector<Aabb3Df>(input->getPointCount());

	// Top level in this list is for objects. Bottom level is for detections that belong to specific objects. Below is an initialization of a helper
	// structure for region growing, which starts with a while-loop.
	std::list<std::list<int>> objectIndices;
	for (int i = 0; i < input->getPointCount(); ++i) {
		objectIndices.emplace_back(1, i);
	}

	int leftIndex = 0;
	while (leftIndex + 1 < objectIndices.size()) {
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
	std::list<ObjectBounds> newObjectBounds;
	for (const auto& separateObjectIndices : objectIndices) {
		auto& objectBounds = newObjectBounds.emplace_back();
		auto entityIdHist = std::unordered_map<Field<ENTITY_ID_I32>::type, int>();
		for (const auto detectionIndex : separateObjectIndices) {
			++entityIdHist[entityIdHostPtr->at(detectionIndex)];
			objectBounds.position += xyzHostPtr->at(detectionIndex);
			objectBounds.aabb += detectionAabbs[detectionIndex];
		}
		// Most common detection entity id is assigned as object id.
		int maxIdCount = -1;
		for (const auto& [entityId, count] : entityIdHist) {
			if (count > maxIdCount) {
				objectBounds.mostCommonEntityId = entityId;
				maxIdCount = count;
			}
		}
		objectBounds.position *= 1 / static_cast<float>(separateObjectIndices.size());
	}

	const auto currentTime = Scene::instance().getTime().value_or(Time::zero()).asMilliseconds();
	const auto deltaTime = currentTime - Scene::instance().getPrevTime().value_or(Time::zero()).asMilliseconds();

	// Check object list from previous frame and try to find matches with newly detected objects.
	// Later, for newly detected objects without match, create new object state.
	for (auto objectStateIt = objectStates.begin(); objectStateIt != objectStates.end();) {
		auto& objectState = *objectStateIt;
		const auto predictedPosition = predictObjectPosition(objectState, deltaTime);
		const auto closestObjectIt = std::min_element(
		    newObjectBounds.cbegin(), newObjectBounds.cend(), [&](const auto& a, const auto& b) {
			    return (a.position - predictedPosition).lengthSquared() < (b.position - predictedPosition).lengthSquared();
		    });

		// There is a newly detected object (current frame) that matches the predicted position of one of objects from previous frame.
		// Update object from previous frame to newly detected object position and remove this positions for next checkouts.
		if (const auto& closestObject = *closestObjectIt;
		    (predictedPosition - closestObject.position).length() < maxMatchingDistance) {
			updateObjectState(objectState, closestObject.position, closestObject.aabb, ObjectStatus::Measured, currentTime,
			                  deltaTime);
			newObjectBounds.erase(closestObjectIt);
			++objectStateIt;
			continue;
		}

		// There is no match for objectState in newly detected object positions. If that object was already detected in previous frame
		// (new, measured or predicted, does not matter) and its last measurement time was within maxPredictionTimeFrame, then its
		// position (and state) in current frame is predicted.
		if (objectState.lastMeasuredTime >= currentTime - maxPredictionTimeFrame) {
			updateObjectState(objectState, predictedPosition, {}, ObjectStatus::Predicted, currentTime, deltaTime);
			++objectStateIt;
			continue;
		}

		// objectState do not have a match in newly detected object positions and it was also on ObjectStatus::Predicted last frame -
		// not this object is considered lost and is removed from object list. Also remove its ID to poll.
		objectIDPoll.push(objectState.id);
		objectStateIt = objectStates.erase(objectStateIt);
	}

	// All newly detected object position that do not have a match in previous frame - create new object state.
	for (const auto& newObject : newObjectBounds) {
		createObjectState(newObject, currentTime);
	}

	updateOutputData();
}

std::vector<rgl_field_t> RadarTrackObjectsNode::getRequiredFieldList() const
{
	return {XYZ_VEC3_F32, DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32, ENTITY_ID_I32};
}

Vec3f RadarTrackObjectsNode::predictObjectPosition(const ObjectState& objectState, double deltaTimeMs) const
{
	assert(objectState.position.getSamplesCount() > 0);
	assert(deltaTimeMs <= std::numeric_limits<float>::max());
	const auto deltaTimeSec = 1e-3f * static_cast<float>(deltaTimeMs);
	const auto assumedVelocity = objectState.absAccel.getSamplesCount() > 0 ?
	                                 (objectState.absVelocity.getLastSample() +
	                                  deltaTimeSec * objectState.absAccel.getLastSample()) :
	                                 objectState.absVelocity.getLastSample();
	const auto predictedMovement = deltaTimeSec * assumedVelocity;
	return objectState.position.getLastSample() + Vec3f{predictedMovement.x(), predictedMovement.y(), 0.0f};
}

void RadarTrackObjectsNode::parseEntityIdToClassProbability(Field<ENTITY_ID_I32>::type entityId,
                                                            ClassificationProbabilities& probabilities)
{
	// May be updated, if entities will be able to belong to multiple classes.
	constexpr auto maxClassificationProbability = std::numeric_limits<uint8_t>::max();

	const auto it = entityIdsToClasses.find(entityId);
	if (it == entityIdsToClasses.cend()) {
		probabilities.classUnknown = maxClassificationProbability;
		return;
	}

	// Single probability is set, but not probabilities are zeroed - the logic here is that probabilities are only set on
	// object creation, so initially all probabilities are zero.
	switch (it->second) {
		case RGL_RADAR_CLASS_CAR: probabilities.classCar = maxClassificationProbability; break;
		case RGL_RADAR_CLASS_TRUCK: probabilities.classTruck = maxClassificationProbability; break;
		case RGL_RADAR_CLASS_MOTORCYCLE: probabilities.classMotorcycle = maxClassificationProbability; break;
		case RGL_RADAR_CLASS_BICYCLE: probabilities.classBicycle = maxClassificationProbability; break;
		case RGL_RADAR_CLASS_PEDESTRIAN: probabilities.classPedestrian = maxClassificationProbability; break;
		case RGL_RADAR_CLASS_ANIMAL: probabilities.classAnimal = maxClassificationProbability; break;
		case RGL_RADAR_CLASS_HAZARD: probabilities.classHazard = maxClassificationProbability; break;
		default: probabilities.classUnknown = maxClassificationProbability;
	}
}

void RadarTrackObjectsNode::createObjectState(const ObjectBounds& objectBounds, double currentTimeMs)
{
	auto& objectState = objectStates.emplace_back();
	if (objectIDPoll.empty()) {
		// Overflow is technically acceptable below, but for > 4,294,967,295 objects it may cause having repeated IDs.
		objectState.id = objectIDCounter++;
	} else {
		objectState.id = objectIDPoll.front();
		objectIDPoll.pop();
	}

	assert(currentTimeMs <= std::numeric_limits<decltype(objectState.creationTime)>::max());
	objectState.creationTime = static_cast<decltype(objectState.creationTime)>(currentTimeMs);
	objectState.lastMeasuredTime = objectState.creationTime;
	objectState.objectStatus = ObjectStatus::New;

	// TODO(Pawel): Consider object radial speed (from detections) as a way to decide here.
	objectState.movementStatus = MovementStatus::Invalid; // No good way to determine it.
	parseEntityIdToClassProbability(objectBounds.mostCommonEntityId, objectState.classificationProbabilities);

	// I do not add velocity or acceleration 0.0f samples because this would affect mean and std dev calculation. However, the
	// number of samples between their stats and position will not match.
	objectState.position.addSample(objectBounds.position);

	// TODO(Pawel): Consider updating this later. One option would be to take rotated bounding box, and calculate orientation as
	// the vector perpendicular to its shorter edge. Then, width would be that defined as that exact edge. The other edge would
	// be taken as length.
	// At this moment I just assume that object length is alongside X axis, and its width is alongside Y axis. Note also that
	// length and width does not have to be correlated to object orientation.
	objectState.length.addSample(objectBounds.aabb.maxCorner().x() - objectBounds.aabb.minCorner().x());
	objectState.width.addSample(objectBounds.aabb.maxCorner().y() - objectBounds.aabb.minCorner().y());
}

void RadarTrackObjectsNode::updateObjectState(ObjectState& objectState, const Vec3f& updatedPosition,
                                              const Aabb3Df& updatedAabb, ObjectStatus objectStatus, double currentTimeMs,
                                              double deltaTimeMs)
{
	assert(deltaTimeMs > 0 && deltaTimeMs <= std::numeric_limits<float>::max());
	const auto displacement = updatedPosition - objectState.position.getLastSample();
	const auto deltaTimeSecInv = 1e3f / static_cast<float>(deltaTimeMs);
	const auto absVelocity = Vec2f{displacement.x() * deltaTimeSecInv, displacement.y() * deltaTimeSecInv};

	const auto radarVelocity = input->getLinearVelocity();
	const auto relVelocity = absVelocity - Vec2f{radarVelocity.x(), radarVelocity.y()};

	if (objectStatus == ObjectStatus::Measured) {
		assert(currentTimeMs <= std::numeric_limits<decltype(objectState.creationTime)>::max());
		objectState.lastMeasuredTime = static_cast<decltype(objectState.creationTime)>(currentTimeMs);
	}
	objectState.objectStatus = objectStatus;
	objectState.movementStatus = displacement.length() > movementSensitivity ? MovementStatus::Moved :
	                                                                           MovementStatus::Stationary;
	objectState.position.addSample(updatedPosition);

	// There has to be at leas one abs velocity sample from previous frames - in other words, this has to be the third frame to be
	// able to calculate acceleration (first frame - position, second frame - velocity, third frame - acceleration).
	if (objectState.absVelocity.getSamplesCount() > 0) {
		const auto absAccel = (absVelocity - objectState.absVelocity.getLastSample()) * deltaTimeSecInv;
		objectState.absAccel.addSample(absAccel);

		const auto relAccel = (relVelocity - objectState.relVelocity.getLastSample()) * deltaTimeSecInv;
		objectState.relAccel.addSample(relAccel);
	}
	objectState.absVelocity.addSample(absVelocity);
	objectState.relVelocity.addSample(relVelocity);

	// Behaves similar to acceleration. In order to calculate orientation I need velocity, which can be calculated starting from
	// the second frame. For this reason, the third frame is the first one when I am able to calculate orientation rate. Additionally,
	// if object does not move, keep its previous orientation.
	const auto orientation = objectState.movementStatus == MovementStatus::Moved ? atan2(absVelocity.y(), absVelocity.x()) :
	                                                                               objectState.orientation.getLastSample();
	if (objectState.orientation.getSamplesCount() > 0) {
		objectState.orientationRate.addSample(orientation - objectState.orientation.getLastSample());
	}
	objectState.orientation.addSample(orientation);

	if (objectStatus == ObjectStatus::Measured) {
		objectState.length.addSample(updatedAabb.maxCorner().x() - updatedAabb.minCorner().x());
		objectState.width.addSample(updatedAabb.maxCorner().y() - updatedAabb.minCorner().y());
	} else {
		objectState.length.addSample(objectState.length.getLastSample());
		objectState.width.addSample(objectState.width.getLastSample());
	}
}

void RadarTrackObjectsNode::updateOutputData()
{
	fieldData[XYZ_VEC3_F32]->resize(objectStates.size(), false, false);
	auto* xyzPtr = static_cast<Field<XYZ_VEC3_F32>::type*>(fieldData[XYZ_VEC3_F32]->getRawWritePtr());

	fieldData[ENTITY_ID_I32]->resize(objectStates.size(), false, false);
	auto* idPtr = static_cast<Field<ENTITY_ID_I32>::type*>(fieldData[ENTITY_ID_I32]->getRawWritePtr());

	int objectIndex = 0;
	for (const auto& objectState : objectStates) {
		assert(objectState.id <= std::numeric_limits<Field<ENTITY_ID_I32>::type>::max());
		xyzPtr[objectIndex] = objectState.position.getLastSample();
		idPtr[objectIndex] = objectState.id;
		++objectIndex;
	}
}
