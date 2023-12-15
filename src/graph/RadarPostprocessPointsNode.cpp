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
#include <graph/NodesPcl.hpp>
#include <gpu/nodeKernels.hpp>

void RadarPostprocessPointsNode::setParameters(float inDistanceSeparation, float inAzimuthSeparation)
{
	this->distanceSeparation = inDistanceSeparation;
	this->azimuthSeparation = inAzimuthSeparation;
}

void RadarPostprocessPointsNode::validateImpl()
{
	IPointsNodeSingleInput::validateImpl();

	// Needed to clear cache because fields in the pipeline may have changed
	// In fact, the cache manager is no longer useful here
	// To be kept/removed in some future refactor (when resolving comment in the `enqueueExecImpl`)
	cacheManager.clear();
}

void RadarPostprocessPointsNode::enqueueExecImpl()
{
	cacheManager.trigger();

	if (input->getPointCount() == 0) {
		filteredIndices->resize(0, false, false);
		return;
	}

	size_t pointCount = input->getPointCount();

	distanceInputHost->copyFrom(input->getFieldData(DISTANCE_F32));
	azimuthInputHost->copyFrom(input->getFieldData(AZIMUTH_F32));

	std::vector<RadarCluster> clusters;
	clusters.emplace_back();
	clusters[0].minDistance = distanceInputHost->getReadPtr()[0];
	clusters[0].maxDistance = distanceInputHost->getReadPtr()[0];
	clusters[0].minAzimuth = azimuthInputHost->getReadPtr()[0];
	clusters[0].maxAzimuth = azimuthInputHost->getReadPtr()[0];
	clusters[0].indices.emplace_back(0);

	for (int i = 1; i < pointCount; ++i) {
		auto thisDistance = distanceInputHost->getReadPtr()[i];
		auto thisAzimuth = azimuthInputHost->getReadPtr()[i];
		bool pointDone = false;
		for (auto&& cluster : clusters) {
			bool distanceInside = thisDistance >= cluster.minDistance && thisDistance <= cluster.maxDistance;
			bool azimuthInside = thisAzimuth >= cluster.minAzimuth && thisAzimuth <= cluster.maxAzimuth;

			if ((azimuthInside || std::abs(thisAzimuth - cluster.minAzimuth) < azimuthSeparation ||
			     std::abs(thisAzimuth - cluster.maxAzimuth) < azimuthSeparation) &&
			    (distanceInside || std::abs(thisDistance - cluster.minDistance) < distanceSeparation ||
			     std::abs(thisDistance - cluster.maxDistance) < distanceSeparation)) {
				cluster.minDistance = std::min(cluster.minDistance, thisDistance);
				cluster.minAzimuth = std::min(cluster.minAzimuth, thisAzimuth);
				cluster.maxDistance = std::max(cluster.maxDistance, thisDistance);
				cluster.maxAzimuth = std::max(cluster.maxAzimuth, thisAzimuth);
				cluster.indices.emplace_back(i);
				pointDone = true;
				break;
			}
		}

		if (pointDone) {
			continue;
		}

		clusters.emplace_back();
		clusters.back().minDistance = thisDistance;
		clusters.back().maxDistance = thisDistance;
		clusters.back().minAzimuth = thisAzimuth;
		clusters.back().maxAzimuth = thisAzimuth;
		clusters.back().indices.emplace_back(i);
	}

	while (1) {
		if (clusters.size() < 2) {
			break;
		}

		bool done = true;

		for (int i = 0; i < clusters.size(); ++i) {
			for (int j = 0; j < clusters.size(); ++j) {
				if (i == j) {
					continue;
				}
				bool distanceInside2 = std::abs(clusters[i].minDistance - clusters[j].maxDistance) <= distanceSeparation &&
				                       std::abs(clusters[i].maxDistance - clusters[j].minDistance) <= distanceSeparation;

				bool azimuthInside2 = std::abs(clusters[i].minAzimuth - clusters[j].maxAzimuth) <= azimuthSeparation &&
				                      std::abs(clusters[i].maxAzimuth - clusters[j].minAzimuth) <= azimuthSeparation;

				if (distanceInside2 && azimuthInside2) {
					clusters[i].minDistance = std::min(clusters[i].minDistance, clusters[j].minDistance);
					clusters[i].minAzimuth = std::min(clusters[i].minAzimuth, clusters[j].minAzimuth);
					clusters[i].maxDistance = std::max(clusters[i].maxDistance, clusters[j].maxDistance);
					clusters[i].maxAzimuth = std::max(clusters[i].maxAzimuth, clusters[j].maxAzimuth);
					clusters[i].indices.insert(clusters[i].indices.begin(), clusters[j].indices.begin(),
					                           clusters[j].indices.end());
					clusters.erase(clusters.begin() + j);
					done = false;
					break;
				}
			}
			if (!done) {
				break;
			}
		}
		if (done) {
			break;
		}
	}

	filteredIndicesHost.clear();
	for (auto&& cluster : clusters) {
		filteredIndicesHost.push_back(cluster.indices[cluster.indices.size() / 2]);
	}

	filteredIndices->copyFromExternal(filteredIndicesHost.data(), filteredIndicesHost.size());

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
			auto msg = fmt::format("SeparateRadarPoints requires its input to be device-accessible, {} is not", field);
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

std::vector<rgl_field_t> RadarPostprocessPointsNode::getRequiredFieldList() const { return {DISTANCE_F32, AZIMUTH_F32}; }
