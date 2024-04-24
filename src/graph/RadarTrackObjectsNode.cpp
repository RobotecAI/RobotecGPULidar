// Copyright 2023 Robotec.AI
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

RadarTrackObjectsNode::RadarTrackObjectsNode() { fieldData.emplace(XYZ_VEC3_F32, createArray<HostPinnedArray>(XYZ_VEC3_F32)); }

void RadarTrackObjectsNode::setParameters(float distanceThreshold, float azimuthThreshold, float elevationThreshold,
                                          float radialSpeedThreshold)
{
	this->distanceThreshold = distanceThreshold;
	this->azimuthThreshold = azimuthThreshold;
	this->elevationThreshold = elevationThreshold;
	this->radialSpeedThreshold = radialSpeedThreshold;
}

void RadarTrackObjectsNode::validateImpl() { IPointsNodeSingleInput::validateImpl(); }

void RadarTrackObjectsNode::enqueueExecImpl()
{
	xyzHostPtr->copyFrom(input->getFieldData(XYZ_VEC3_F32));
	distanceHostPtr->copyFrom(input->getFieldData(DISTANCE_F32));
	azimuthHostPtr->copyFrom(input->getFieldData(AZIMUTH_F32));
	elevationHostPtr->copyFrom(input->getFieldData(ELEVATION_F32));
	radialSpeedHostPtr->copyFrom(input->getFieldData(RADIAL_SPEED_F32));

	const int detectionsCount = input->getPointCount();

	objectStates.clear(); // TODO(Pawel): Remove this when we start working on tracking.

	struct TrackedObject
	{
		Vec3f position;
		float radialSpeed;
	};
	std::vector<TrackedObject> trackedObjects;
	std::vector<int> objectIndexLookup(detectionsCount, -1);

	std::list<std::list<int>> objectIndices;
	for (auto i = 0; i < detectionsCount; ++i) {
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

	fieldData[XYZ_VEC3_F32]->resize(objectIndices.size(), true, false);
	auto* xyzPtr = static_cast<Vec3f*>(fieldData[XYZ_VEC3_F32]->getRawWritePtr());
	uint32_t objectIndex = 0;

	for (const auto& separateObjectIndices : objectIndices) {
		auto objectCenter = Vec3f(0.0f, 0.0f, 0.0f);
		for (const auto index : separateObjectIndices) {
			objectCenter += xyzHostPtr->at(index);
		}

		auto& objectState = objectStates.emplace_back();
		objectState.id = objectIndex++;
		objectState.framesCount = 1; // This will be updated with object tracking introduction.
		objectState.creationTime = static_cast<uint32_t>(Scene::instance().getTime().value_or(Time::zero()).asMilliseconds());
		objectState.objectStatus = ObjectStatus::New;
		objectState.movementStatus =
		    MovementStatus::Invalid; // Newly created object does not have velocity data (no 'previous frame' exists).
		objectState.position.addSample(objectCenter / static_cast<float>(separateObjectIndices.size()));

		// TODO(Pawel): This will require modyfing radar postprocess node - I will require some bounding box here.
		objectState.orientation.addSample(0.0f);
		objectState.absVelocity.addSample(Vec2f(0.0f, 0.0f));
		objectState.relVelocity.addSample(Vec2f(0.0f, 0.0f));
		objectState.absAccel.addSample(Vec2f(0.0f, 0.0f));
		objectState.relAccel.addSample(Vec2f(0.0f, 0.0f));
		objectState.orientationRate.addSample(0.0f);
		objectState.length.addSample(0.0f);
		objectState.width.addSample(0.0f);

		// TODO(Pawel): Be careful with indexing here, when working on tracking - id will not match the index for not new objects.
		xyzPtr[objectState.id] = 1 / static_cast<float>(separateObjectIndices.size()) * objectCenter;
	}
}

std::vector<rgl_field_t> RadarTrackObjectsNode::getRequiredFieldList() const
{
	return {XYZ_VEC3_F32, DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32};
}
