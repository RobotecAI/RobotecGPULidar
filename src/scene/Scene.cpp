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

#include <scene/Scene.hpp>
#include <scene/Entity.hpp>
#include <memory/DeviceSyncArray.hpp>
#include <memory/HostPinnedArray.hpp>

API_OBJECT_INSTANCE(Scene);

std::shared_ptr<Scene> Scene::defaultInstance()
{
	static auto scene = Scene::create();
	return scene;
}

Scene::Scene() : stream(CudaStream::create(cudaStreamNonBlocking)) { }

std::size_t Scene::getObjectCount()
{ return entities.size(); }

void Scene::clear()
{
	entities.clear();
	requestFullRebuild();
}

void Scene::addEntity(std::shared_ptr<Entity> entity)
{
	entity->scene = weak_from_this();
	entities.insert(entity);
	requestFullRebuild();
}

void Scene::removeEntity(std::shared_ptr<Entity> entity)
{
	entities.erase(entity);
	requestFullRebuild();
}

void Scene::requestFullRebuild()
{
	requestASRebuild();
	requestSBTRebuild();
}

OptixTraversableHandle Scene::getAS()
{
	if (!cachedAS.has_value()) {
		cachedAS = buildAS();
	}
	return *cachedAS;
}

OptixShaderBindingTable Scene::getSBT()
{
	if (!cachedSBT.has_value()) {
		cachedSBT = buildSBT();
	}
	return *cachedSBT;
}

OptixShaderBindingTable Scene::buildSBT()
{
	static DeviceSyncArray<HitgroupRecord>::Ptr dHitgroupRecords = DeviceSyncArray<HitgroupRecord>::create();
	static DeviceSyncArray<RaygenRecord>::Ptr dRaygenRecords = DeviceSyncArray<RaygenRecord>::create();
	static DeviceSyncArray<MissRecord>::Ptr dMissRecords = DeviceSyncArray<MissRecord>::create();
	static HostPinnedArray<HitgroupRecord>::Ptr hHitgroupRecords = HostPinnedArray<HitgroupRecord>::create();

	// TODO(prybicki): low priority: can HG count be reduced to be == count(GASes)? or must it be count(IASes)?

	hHitgroupRecords->reserve(entities.size(), false);
	hHitgroupRecords->clear(false);
	for (auto&& entity : entities) {
		auto& mesh = entity->mesh;
		hHitgroupRecords->append(HitgroupRecord {
			.data = {
				.vertex = mesh->dVertices->getReadPtr(),
				.index = mesh->dIndices->getReadPtr(),
				.vertex_count = mesh->dVertices->getCount(),
				.index_count = mesh->dIndices->getCount(),
			}
		});
		HitgroupRecord& last = hHitgroupRecords->at(hHitgroupRecords->getCount()-1);
		CHECK_OPTIX(optixSbtRecordPackHeader(Optix::getOrCreate().hitgroupPG, last.header));
	}
	dHitgroupRecords->copyFrom(hHitgroupRecords);

	RaygenRecord hRaygenRecord = {};
	CHECK_OPTIX(optixSbtRecordPackHeader(Optix::getOrCreate().raygenPG, &hRaygenRecord));
	dRaygenRecords->copyFromHost(&hRaygenRecord, 1);

	MissRecord hMissRecord = {};
	CHECK_OPTIX(optixSbtRecordPackHeader(Optix::getOrCreate().missPG, &hMissRecord));
	dMissRecords->copyFromHost(&hMissRecord, 1);

	return OptixShaderBindingTable{
		.raygenRecord = dRaygenRecords->getDeviceReadPtr(),
		.missRecordBase = dMissRecords->getDeviceReadPtr(),
		.missRecordStrideInBytes = sizeof(MissRecord),
		.missRecordCount = 1U,
		.hitgroupRecordBase = getObjectCount() > 0 ? dHitgroupRecords->getDeviceReadPtr() : static_cast<CUdeviceptr>(0),
		.hitgroupRecordStrideInBytes = sizeof(HitgroupRecord),
		.hitgroupRecordCount = static_cast<unsigned>(dHitgroupRecords->getCount()),
	};
}

OptixTraversableHandle Scene::buildAS()
{
	if (getObjectCount() == 0) {
		return static_cast<OptixTraversableHandle>(0);
	}
	auto instances = HostPinnedArray<OptixInstance>::create();
	instances->reserve(entities.size(), false);
	for (auto&& entity : entities) {
		// TODO(prybicki): this is somewhat inefficient, because most of the time only transform changes.
		instances->append(entity->getIAS(static_cast<int>(instances->getCount())));
	}

	// *** *** *** ACHTUNG *** *** ***
	// Calls to cudaMemcpy below are a duck-tape for synchronizing all streams from LidarContexts.

	dInstances->resize(instances->getCount(), false, false);
	dInstances->copyFrom(instances);

	OptixBuildInput instanceInput = {
	.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES,
	.instanceArray = {
	.instances = dInstances->getDeviceReadPtr(),
	.numInstances = static_cast<unsigned int>(dInstances->getCount())
	},
	};

	OptixAccelBuildOptions accelBuildOptions = {
	.buildFlags =
	OPTIX_BUILD_FLAG_ALLOW_UPDATE // TODO(prybicki): figure out if there's a faster way to update than the current one
	| OPTIX_BUILD_FLAG_ALLOW_COMPACTION,
	.operation = OPTIX_BUILD_OPERATION_BUILD
	};

	scratchpad.resizeToFit(instanceInput, accelBuildOptions);

	OptixAccelEmitDesc emitDesc = {
	.result = scratchpad.dCompactedSize->getDeviceReadPtr(),
	.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
	};

	OptixTraversableHandle sceneHandle;
	CHECK_OPTIX(optixAccelBuild(Optix::getOrCreate().context,
	                            stream->getHandle(),
	                            &accelBuildOptions,
	                            &instanceInput,
	                            1,
	                            scratchpad.dTemp->getDeviceReadPtr(),
	                            scratchpad.dTemp->getSizeOf() * scratchpad.dTemp->getCount(),
	                            scratchpad.dFull->getDeviceReadPtr(),
	                            scratchpad.dFull-> getSizeOf() * scratchpad.dFull->getCount(),
	                            &sceneHandle,
	                            &emitDesc,
	                            1
	));

	CHECK_CUDA(cudaStreamSynchronize(stream->getHandle()));

	// scratchpad.doCompaction(sceneHandle);

	return sceneHandle;
}

void Scene::requestASRebuild()
{
	cachedAS.reset();
}

void Scene::requestSBTRebuild()
{
	cachedSBT.reset();
}

CudaStream::Ptr Scene::getStream()
{
	return stream;
}


