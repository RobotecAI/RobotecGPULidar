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

#include <algorithm>
#include <numbers>
#include <complex>

#include <repr.hpp>
#include <graph/NodesCore.hpp>
#include <gpu/nodeKernels.hpp>

inline static std::optional<rgl_radar_scope_t> getRadarScopeWithinDistance(const std::vector<rgl_radar_scope_t>& radarScopes,
                                                                           Field<DISTANCE_F32>::type distance)
{
	for (auto&& scope : radarScopes) {
		if (scope.begin_distance <= distance && distance <= scope.end_distance) {
			return scope;
		}
	}
	return std::nullopt;
}

void RadarPostprocessPointsNode::setParameters(const std::vector<rgl_radar_scope_t>& radarScopes, float rayAzimuthStepRad,
                                               float rayElevationStepRad, float frequency, float powerTransmitted,
                                               float cumulativeDeviceGain, float receivedNoiseMean, float receivedNoiseStDev)
{
	this->rayAzimuthStepRad = rayAzimuthStepRad;
	this->rayElevationStepRad = rayElevationStepRad;
	this->frequencyHz = frequency;
	this->radarScopes = radarScopes;
	this->powerTransmittedDbm = powerTransmitted;
	this->cumulativeDeviceGainDbi = cumulativeDeviceGain;
	this->receivedNoiseMeanDb = receivedNoiseMean;
	this->receivedNoiseStDevDb = receivedNoiseStDev;
}

void RadarPostprocessPointsNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();

	if (!input->isDense()) {
		throw InvalidPipeline("RadarComputeEnergyPointsNode requires dense input");
	}

	// Needed to clear cache because fields in the pipeline may have changed
	// In fact, the cache manager is no longer useful here
	// To be kept/removed in some future refactor (when resolving comment in the `enqueueExecImpl`)
	cacheManager.clear();
}

void RadarPostprocessPointsNode::enqueueExecImpl()
{
	cacheManager.trigger();

	auto raysPtr = input->getFieldDataTyped<RAY_POSE_MAT3x4_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto distancePtr = input->getFieldDataTyped<DISTANCE_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto normalPtr = input->getFieldDataTyped<NORMAL_VEC3_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	auto xyzPtr = input->getFieldDataTyped<XYZ_VEC3_F32>()->asSubclass<DeviceAsyncArray>()->getReadPtr();
	outBUBRFactorDev->resize(input->getPointCount(), false, false);
	gpuRadarComputeEnergy(getStreamHandle(), input->getPointCount(), rayAzimuthStepRad, rayElevationStepRad, frequencyHz,
	                      input->getLookAtOriginTransform(), raysPtr, distancePtr, normalPtr, xyzPtr,
	                      outBUBRFactorDev->getWritePtr());
	outBUBRFactorHost->copyFrom(outBUBRFactorDev);
	CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));

	if (input->getPointCount() == 0) {
		filteredIndices->resize(0, false, false);
		return;
	}

	xyzInputHost->copyFrom(input->getFieldData(XYZ_VEC3_F32));
	distanceInputHost->copyFrom(input->getFieldData(DISTANCE_F32));
	azimuthInputHost->copyFrom(input->getFieldData(AZIMUTH_F32));
	radialSpeedInputHost->copyFrom(input->getFieldData(RADIAL_SPEED_F32));
	elevationInputHost->copyFrom(input->getFieldData(ELEVATION_F32));

	std::vector<RadarCluster> clusters;
	for (int i = 0; i < input->getPointCount(); ++i) {
		const auto distance = distanceInputHost->at(i);
		const auto azimuth = azimuthInputHost->at(i);
		const auto radialSpeed = radialSpeedInputHost->at(i);
		const auto elevation = elevationInputHost->at(i);
		bool isPointClustered = false;
		const auto radarScope = getRadarScopeWithinDistance(radarScopes, distance);
		if (!radarScope.has_value()) {
			continue;
		}
		for (auto&& cluster : clusters) {
			if (cluster.isCandidate(distance, azimuth, radialSpeed, radarScope.value())) {
				cluster.addPoint(i, distance, azimuth, radialSpeed, elevation);
				isPointClustered = true;
				break;
			}
		}

		if (!isPointClustered) {
			// Create a new cluster
			clusters.emplace_back(i, distance, azimuth, radialSpeed, elevation);
		}
	}

	// Merge clusters if are close enough
	bool allClustersGood = false;
	while (clusters.size() > 1 && !allClustersGood) {
		allClustersGood = true;
		for (int i = 0; i < clusters.size(); ++i) {
			for (int j = i + 1; j < clusters.size(); ++j) {
				if (clusters[i].canMergeWith(clusters[j], radarScopes)) {
					clusters[i].takeIndicesFrom(std::move(clusters[j]));
					clusters.erase(clusters.begin() + j);
					allClustersGood = false;
					break;
				}
			}
			if (!allClustersGood) {
				break;
			}
		}
	}

	filteredIndicesHost.clear();
	for (auto&& cluster : clusters) {
		filteredIndicesHost.push_back(
		    cluster.findDirectionalCenterIndex(azimuthInputHost->getReadPtr(), elevationInputHost->getReadPtr()));
	}

	filteredIndices->copyFromExternal(filteredIndicesHost.data(), filteredIndicesHost.size());

	const auto lambda = 299'792'458.0f / frequencyHz;
	const auto lambdaSqrtDbsm = 10.0f * log10f(lambda * lambda);

	// Compute per-cluster properties
	clusterRcsHost->resize(filteredIndicesHost.size(), false, false);
	clusterPowerHost->resize(filteredIndicesHost.size(), false, false);
	clusterNoiseHost->resize(filteredIndicesHost.size(), false, false);
	clusterSnrHost->resize(filteredIndicesHost.size(), false, false);

	std::normal_distribution<float> gaussianNoise(receivedNoiseMeanDb, receivedNoiseStDevDb);
	clusterAabbs.resize(clusters.size());

	for (int clusterIdx = 0; clusterIdx < clusters.size(); ++clusterIdx) {
		std::complex<float> AU = 0;
		std::complex<float> AR = 0;
		auto& cluster = clusters[clusterIdx];
		auto& clusterAabb = clusterAabbs[clusterIdx];
		clusterAabb.reset();

		for (const auto pointInCluster : cluster.indices) {
			clusterAabb.expand(xyzInputHost->at(pointInCluster));
			std::complex<float> BU = {outBUBRFactorHost->at(pointInCluster)[0].real(),
			                          outBUBRFactorHost->at(pointInCluster)[0].imag()};
			std::complex<float> BR = {outBUBRFactorHost->at(pointInCluster)[1].real(),
			                          outBUBRFactorHost->at(pointInCluster)[1].imag()};
			std::complex<float> factor = {outBUBRFactorHost->at(pointInCluster)[2].real(),
			                              outBUBRFactorHost->at(pointInCluster)[2].imag()};
			AU += BU * factor;
			AR += BR * factor;
		}

		// https://en.wikipedia.org/wiki/Radar_cross_section#Formulation
		const auto rcsDbsm = 10.0f * log10f(4.0f * std::numbers::pi_v<float> * (powf(abs(AU), 2) + powf(abs(AR), 2)));

		// TODO: Handle nans in RCS.
		if (std::isnan(rcsDbsm)) {
			throw InvalidPipeline("RCS is NaN");
		}

		const auto distance = distanceInputHost->at(filteredIndicesHost.at(clusterIdx));
		const auto multiplier = 10.0f * log10f(powf(4 * std::numbers::pi_v<float>, 3)) + 10.0f * log10f(powf(distance, 4));

		const auto powerReceived = powerTransmittedDbm + cumulativeDeviceGainDbi + cumulativeDeviceGainDbi + rcsDbsm +
		                           lambdaSqrtDbsm - multiplier;

		clusterRcsHost->at(clusterIdx) = rcsDbsm;
		clusterNoiseHost->at(clusterIdx) = gaussianNoise(randomDevice);
		clusterPowerHost->at(clusterIdx) = powerReceived + clusterNoiseHost->at(clusterIdx); // power received + noise
		clusterSnrHost->at(clusterIdx) = clusterPowerHost->at(clusterIdx) - clusterNoiseHost->at(clusterIdx);
	}

	clusterPowerDev->copyFrom(clusterPowerHost);
	clusterRcsDev->copyFrom(clusterRcsHost);
	clusterNoiseDev->copyFrom(clusterNoiseHost);
	clusterSnrDev->copyFrom(clusterSnrHost);

	// getFieldData may be called in client's thread from rgl_graph_get_result_data
	// Doing job there would be:
	// - unexpected (job was supposed to be done asynchronously)
	// - hard to implement:
	//     - to avoid blocking on yet-running graph stream, we would need do it in copy stream, which would require
	//       temporary rebinding DAAs to copy stream, which seems like nightmarish idea
	// Therefore, once we know what fields are requested, we compute them eagerly
	// This is supposed to be removed in some future refactor (e.g. when introducing LayeredSoA)
	for (auto&& field : cacheManager.getKeys()) {
		getFieldData(field);
	}
}

size_t RadarPostprocessPointsNode::getWidth() const
{
	this->synchronize();
	return filteredIndices->getCount();
}

IAnyArray::ConstPtr RadarPostprocessPointsNode::getFieldData(rgl_field_t field)
{
	std::lock_guard lock{getFieldDataMutex};

	if (field == POWER_F32) {
		return clusterPowerDev->asAny();
	}
	if (field == RCS_F32) {
		return clusterRcsDev->asAny();
	}
	if (field == NOISE_F32) {
		return clusterNoiseDev->asAny();
	}
	if (field == SNR_F32) {
		return clusterSnrDev->asAny();
	}

	if (!cacheManager.contains(field)) {
		auto fieldData = createArray<DeviceAsyncArray>(field, arrayMgr);
		fieldData->resize(filteredIndices->getCount(), false, false);
		cacheManager.insert(field, fieldData, true);
	}

	if (!cacheManager.isLatest(field)) {
		auto fieldData = cacheManager.getValue(field);
		fieldData->resize(filteredIndices->getCount(), false, false);
		char* outPtr = static_cast<char*>(fieldData->getRawWritePtr());
		auto fieldArray = input->getFieldData(field);
		if (!isDeviceAccessible(fieldArray->getMemoryKind())) {
			auto msg = fmt::format("RadarPostprocessPoints requires its input to be device-accessible, {} is not", field);
			throw InvalidPipeline(msg);
		}
		const char* inputPtr = static_cast<const char*>(fieldArray->getRawReadPtr());
		gpuFilter(getStreamHandle(), filteredIndices->getCount(), filteredIndices->getReadPtr(), outPtr, inputPtr,
		          getFieldSize(field));
		CHECK_CUDA(cudaStreamSynchronize(getStreamHandle()));
		cacheManager.setUpdated(field);
	}

	return cacheManager.getValue(field);
}

std::vector<rgl_field_t> RadarPostprocessPointsNode::getRequiredFieldList() const
{
	return {DISTANCE_F32, AZIMUTH_F32, ELEVATION_F32, RADIAL_SPEED_F32, RAY_POSE_MAT3x4_F32, NORMAL_VEC3_F32, XYZ_VEC3_F32};
}

// RadarCluster methods implementation

RadarPostprocessPointsNode::RadarCluster::RadarCluster(Field<RAY_IDX_U32>::type index, float distance, float azimuth,
                                                       float radialSpeed, float elevation)
{
	indices.emplace_back(index);
	minMaxDistance = {distance, distance};
	minMaxAzimuth = {azimuth, azimuth};
	minMaxRadialSpeed = {radialSpeed, radialSpeed};
	minMaxElevation = {elevation, elevation};
}

void RadarPostprocessPointsNode::RadarCluster::addPoint(Field<RAY_IDX_U32>::type index, float distance, float azimuth,
                                                        float radialSpeed, float elevation)
{
	indices.emplace_back(index);
	minMaxDistance[0] = std::min(minMaxDistance[0], distance);
	minMaxDistance[1] = std::max(minMaxDistance[1], distance);
	minMaxAzimuth[0] = std::min(minMaxAzimuth[0], azimuth);
	minMaxAzimuth[1] = std::max(minMaxAzimuth[1], azimuth);

	// There are three reasonable cases that should be handled as intended:
	// - limit and radialSpeed are nans - limit will be set to nan (from radialSpeed)
	// - limit is nan and radialSpeed if a number - limit will be set to a number (from radialSpeed)
	// - limit and radialSpeed are numbers - std::min will be called.
	// There can technically be a fourth case:
	// - limit is a number and radialSpeed is nan - this should technically be handled via order of arguments in std::min
	//   (assumption that comparison inside will return false); what is more important, such candidate should be eliminated
	//   with isCandidate method;
	// The fourth case should not occur - radial speed is either nan for all entities (first raycast, with delta time = 0) or
	// has value for all entities.
	minMaxRadialSpeed[0] = std::isnan(minMaxRadialSpeed[0]) ? radialSpeed : std::min(minMaxRadialSpeed[0], radialSpeed);
	minMaxRadialSpeed[1] = std::isnan(minMaxRadialSpeed[1]) ? radialSpeed : std::max(minMaxRadialSpeed[1], radialSpeed);

	minMaxElevation[0] = std::min(minMaxElevation[0], elevation);
	minMaxElevation[1] = std::max(minMaxElevation[1], elevation);
}

inline bool RadarPostprocessPointsNode::RadarCluster::isCandidate(float distance, float azimuth, float radialSpeed,
                                                                  const rgl_radar_scope_t& radarScope) const
{
	const auto isWithinDistanceLowerBound = distance + radarScope.distance_separation_threshold >= minMaxDistance[0];
	const auto isWithinDistanceUpperBound = distance - radarScope.distance_separation_threshold <= minMaxDistance[1];
	const auto isWithinAzimuthLowerBound = azimuth + radarScope.azimuth_separation_threshold >= minMaxAzimuth[0];
	const auto isWithinAzimuthUpperBound = azimuth - radarScope.azimuth_separation_threshold <= minMaxAzimuth[1];

	const auto isWithinRadialSpeedLowerBound = radialSpeed + radarScope.radial_speed_separation_threshold >=
	                                           minMaxRadialSpeed[0];
	const auto isWithinRadialSpeedUpperBound = radialSpeed - radarScope.radial_speed_separation_threshold <=
	                                           minMaxRadialSpeed[1];

	// Radial speed may be nan if this is the first raytracing call (delta time equals 0). When cluster has nan radial speed
	// limits (technically just do not have radial speed information), the goal below is to ignore radial speed checkout. The
	// next goal is to allow adding points to cluster, if these points have non nan radial speed - to eliminate undefined
	// radial speed information from cluster. This should also work well, when limits are fine and candidate's radial speed
	// is nan - then radial speed checkouts will give false and the candidate will not pass.
	// Context for radial speed checkouts below - this can be interpreted as "true if any radial speed limit is nan or
	// candidate's radial speed is within limits"
	return isWithinDistanceLowerBound && isWithinDistanceUpperBound && isWithinAzimuthLowerBound && isWithinAzimuthUpperBound &&
	       (std::isunordered(minMaxRadialSpeed[0], minMaxRadialSpeed[1]) ||
	        (isWithinRadialSpeedLowerBound && isWithinRadialSpeedUpperBound));
}

inline bool RadarPostprocessPointsNode::RadarCluster::canMergeWith(const RadarCluster& other,
                                                                   const std::vector<rgl_radar_scope_t>& radarScopes) const
{
	// Helper functions
	auto doRangesOverlap = [](const Vec2f& a, const Vec2f& b) {
		return (b[0] <= a[1] && b[1] >= a[0]) || (a[0] <= b[1] && a[1] >= b[0]);
	};
	auto areRangesWithinThreshold = [](const Vec2f& a, const Vec2f& b, float threshold) {
		return std::abs(a[1] - b[0]) <= threshold || std::abs(b[1] - a[0]) <= threshold;
	};

	// Find distances that will be compared with each other:
	// |---cluster1---|    |---cluster2---|
	//                ^    ^
	const float minDistanceToCompare = std::max(minMaxDistance[0], other.minMaxDistance[0]);
	const float maxDistanceToCompare = std::min(minMaxDistance[1], other.minMaxDistance[1]);
	const auto radarScope = getRadarScopeWithinDistance(radarScopes, std::max(minDistanceToCompare, maxDistanceToCompare));

	assert(radarScope.has_value()); // Must have value because it was already checked when creating clusters

	bool isDistanceGood = doRangesOverlap(minMaxDistance, other.minMaxDistance) ||
	                      areRangesWithinThreshold(minMaxDistance, other.minMaxDistance,
	                                               radarScope->distance_separation_threshold);

	bool isAzimuthGood = doRangesOverlap(minMaxAzimuth, other.minMaxAzimuth) ||
	                     areRangesWithinThreshold(minMaxAzimuth, other.minMaxAzimuth, radarScope->azimuth_separation_threshold);

	bool isRadialSpeedGood = doRangesOverlap(minMaxRadialSpeed, other.minMaxRadialSpeed) ||
	                         areRangesWithinThreshold(minMaxRadialSpeed, other.minMaxRadialSpeed,
	                                                  radarScope->radial_speed_separation_threshold);

	// Radial speed check is ignored if one of limits on it, in one of clusters, is nan.
	return isDistanceGood && isAzimuthGood &&
	       (isRadialSpeedGood || std::isunordered(minMaxRadialSpeed[0], minMaxRadialSpeed[1]) ||
	        std::isunordered(other.minMaxRadialSpeed[0], other.minMaxRadialSpeed[1]));
}

void RadarPostprocessPointsNode::RadarCluster::takeIndicesFrom(RadarCluster&& other)
{
	minMaxDistance[0] = std::min(minMaxDistance[0], other.minMaxDistance[0]);
	minMaxDistance[1] = std::max(minMaxDistance[1], other.minMaxDistance[1]);
	minMaxAzimuth[0] = std::min(minMaxAzimuth[0], other.minMaxAzimuth[0]);
	minMaxAzimuth[1] = std::max(minMaxAzimuth[1], other.minMaxAzimuth[1]);
	minMaxRadialSpeed[0] = std::isnan(other.minMaxRadialSpeed[0]) ? minMaxRadialSpeed[0] :
	                                                                std::min(other.minMaxRadialSpeed[0], minMaxRadialSpeed[0]);
	minMaxRadialSpeed[1] = std::isnan(other.minMaxRadialSpeed[1]) ? minMaxRadialSpeed[1] :
	                                                                std::min(other.minMaxRadialSpeed[1], minMaxRadialSpeed[1]);
	minMaxElevation[0] = std::min(minMaxElevation[0], other.minMaxElevation[0]);
	minMaxElevation[1] = std::max(minMaxElevation[1], other.minMaxElevation[1]);

	// Move indices
	std::size_t n = indices.size();
	indices.resize(indices.size() + other.indices.size());
	std::move(other.indices.begin(), other.indices.end(), indices.begin() + n);
}

Field<RAY_IDX_U32>::type RadarPostprocessPointsNode::RadarCluster::findDirectionalCenterIndex(
    const Field<AZIMUTH_F32>::type* azimuths, const Field<ELEVATION_F32>::type* elevations) const
{
	auto meanAzimuth = (minMaxAzimuth[0] + minMaxAzimuth[1]) / 2.0f;
	auto meanElevation = (minMaxElevation[0] + minMaxElevation[1]) / 2.0f;

	float minDistance = FLT_MAX;
	uint32_t minIndex = indices.front();

	for (auto&& i : indices) {
		float distance = std::abs(azimuths[i] - meanAzimuth) + std::abs(elevations[i] - meanElevation);
		if (distance < minDistance) {
			minDistance = distance;
			minIndex = i;
		}
	}
	return minIndex;
}
