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

API_OBJECT_INSTANCE(Scene);

std::shared_ptr<Scene> Scene::defaultInstance()
{
	static auto scene = Scene::create();
	return scene;
}

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
	static DeviceBuffer<HitgroupRecord> dHitgroupRecords;
	static DeviceBuffer<RaygenRecord> dRaygenRecords;
	static DeviceBuffer<MissRecord> dMissRecords;

	// TODO(prybicki): low priority: can HG count be reduced to be == count(GASes)? or must it be count(IASes)?
	std::vector<HitgroupRecord> hHitgroupRecords;
	for (auto&& entity : entities) {
		auto& mesh = entity->mesh;
		hHitgroupRecords.emplace_back(); // TODO(prybicki): fix, this is weird
		HitgroupRecord *hr = &(*hHitgroupRecords.rbegin());
		CHECK_OPTIX(optixSbtRecordPackHeader(Optix::instance().hitgroupPG, hr));
		TriangleMeshSBTData data{};
		data.vertex = mesh->dVertices.readDevice();
		data.index = mesh->dIndices.readDevice();
		data.vertex_count = mesh->dVertices.getElemCount();
		data.index_count = mesh->dIndices.getElemCount();
		hr->data = data;
	}
	dHitgroupRecords.copyFromHost(hHitgroupRecords);

	RaygenRecord hRaygenRecord;
	CHECK_OPTIX(optixSbtRecordPackHeader(Optix::instance().raygenPG, &hRaygenRecord));
	dRaygenRecords.copyFromHost(&hRaygenRecord, 1);

	MissRecord hMissRecord;
	CHECK_OPTIX(optixSbtRecordPackHeader(Optix::instance().missPG, &hMissRecord));
	dMissRecords.copyFromHost(&hMissRecord, 1);

	OptixShaderBindingTable outSBT{};
	outSBT.raygenRecord = dRaygenRecords.readDeviceRaw();
	outSBT.missRecordBase = dMissRecords.readDeviceRaw();
	outSBT.missRecordStrideInBytes = sizeof(MissRecord);
	outSBT.missRecordCount = 1U;
	outSBT.hitgroupRecordBase = getObjectCount() > 0 ? dHitgroupRecords.readDeviceRaw() : static_cast<CUdeviceptr>(0);
	outSBT.hitgroupRecordStrideInBytes = sizeof(HitgroupRecord);
	outSBT.hitgroupRecordCount = static_cast<unsigned>(dHitgroupRecords.getElemCount());
	return outSBT;
}

OptixTraversableHandle Scene::buildAS()
{
	if (getObjectCount() == 0) {
		return static_cast<OptixTraversableHandle>(0);
	}
	std::vector<OptixInstance> instances;
	for (auto&& entity : entities) {
		// TODO(prybicki): this is somewhat inefficient, because most of the time only transform changes.
		instances.push_back(entity->getIAS(static_cast<int>(instances.size())));
	}

	// *** *** *** ACHTUNG *** *** ***
	// Calls to cudaMemcpy below are a duck-tape for synchronizing all streams from LidarContexts.
	dInstances.resizeToFit(instances.size());
	dInstances.copyFromHost(instances);

	OptixBuildInputInstanceArray instanceArray{};
	instanceArray.instances = dInstances.readDeviceRaw();
	instanceArray.numInstances = static_cast<unsigned int>(dInstances.getElemCount());

	OptixBuildInput instanceInput{};
	instanceInput.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
	instanceInput.instanceArray = instanceArray;

	OptixAccelBuildOptions accelBuildOptions{};
	accelBuildOptions.buildFlags =
	    OPTIX_BUILD_FLAG_ALLOW_UPDATE // TODO(prybicki): figure out if there's a faster way to update than the current one
	    | OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
	accelBuildOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

	scratchpad.resizeToFit(instanceInput, accelBuildOptions);

	OptixAccelEmitDesc emitDesc{};
	emitDesc.result = scratchpad.dCompactedSize.readDeviceRaw();
	emitDesc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;

	OptixTraversableHandle sceneHandle;
	CHECK_OPTIX(optixAccelBuild(Optix::instance().context,
	                            nullptr, // TODO(prybicki): run in stream
	                            &accelBuildOptions,
	                            &instanceInput,
	                            1,
	                            scratchpad.dTemp.readDeviceRaw(),
	                            scratchpad.dTemp.getByteSize(),
	                            scratchpad.dFull.readDeviceRaw(),
	                            scratchpad.dFull.getByteSize(),
	                            &sceneHandle,
	                            &emitDesc,
	                            1
	));

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
