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
#include <scene/Texture.hpp>
#include <memory/Array.hpp>

Scene& Scene::instance()
{
	static Scene scene;
	return scene;
}

Scene::Scene() : stream(CudaStream::create(cudaStreamNonBlocking)) {}

std::size_t Scene::getObjectCount() const { return entities.size(); }

void Scene::clear()
{
	entities.clear();
	requestASRebuild();
	requestSBTRebuild();
}

void Scene::addEntity(std::shared_ptr<Entity> entity)
{
	entities.insert(entity);
	requestASRebuild();
	requestSBTRebuild();
}

void Scene::removeEntity(std::shared_ptr<Entity> entity)
{
	entities.erase(entity);
	requestASRebuild();
	requestSBTRebuild();
}

OptixTraversableHandle Scene::getASLocked()
{
	std::lock_guard optixStructsLock(optixStructsMutex);
	if (!cachedAS.has_value()) {
		cachedAS = buildAS();
	}
	return *cachedAS;
}

OptixShaderBindingTable Scene::getSBTLocked()
{
	std::lock_guard optixStructsLock(optixStructsMutex);
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
		hHitgroupRecords->append(HitgroupRecord{
		    .data = {
		             .vertex = mesh->dVertices->getReadPtr(),
		             .index = mesh->dIndices->getReadPtr(),
		             .vertexCount = mesh->dVertices->getCount(),
		             .indexCount = mesh->dIndices->getCount(),
		             .textureCoords = mesh->dTextureCoords.has_value() ? mesh->dTextureCoords.value()->getReadPtr() : nullptr,
		             .textureCoordsCount = mesh->dTextureCoords.has_value() ? mesh->dTextureCoords.value()->getCount() : 0,
		             .texture = entity->intensityTexture != nullptr ? entity->intensityTexture->getTextureObject() : 0,
		             }
        });
		HitgroupRecord& last = hHitgroupRecords->at(hHitgroupRecords->getCount() - 1);
		CHECK_OPTIX(optixSbtRecordPackHeader(Optix::getOrCreate().hitgroupPG, last.header));
	}
	dHitgroupRecords->copyFrom(hHitgroupRecords);

	RaygenRecord hRaygenRecord = {};
	CHECK_OPTIX(optixSbtRecordPackHeader(Optix::getOrCreate().raygenPG, &hRaygenRecord));
	dRaygenRecords->copyFromExternal(&hRaygenRecord, 1);

	MissRecord hMissRecord = {};
	CHECK_OPTIX(optixSbtRecordPackHeader(Optix::getOrCreate().missPG, &hMissRecord));
	dMissRecords->copyFromExternal(&hMissRecord, 1);

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

	// Construct Instance Acceleration Structures based on Entities present on the scene
	// TODO: this approach may be inefficient - most of the time only transform changes
	auto instances = HostPinnedArray<OptixInstance>::create();
	instances->reserve(entities.size(), false);
	for (auto&& entity : entities) {
		int idx = instances->getCount();
		OptixInstance instance = {
		    .instanceId = static_cast<unsigned int>(entity->id),
		    // (more efficient), instead of storing it in HitGroupRecord
		    .sbtOffset = static_cast<unsigned int>(idx), // NOTE: this assumes a single SBT record per GAS
		    .visibilityMask = 255,
		    .flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT,
		    .traversableHandle = entity->mesh->getGAS(getStream()),
		};
		entity->transform.toRaw(instance.transform);
		instances->append(instance);
	}

	dInstances->resize(instances->getCount(), false, false);
	dInstances->copyFrom(instances);

	OptixBuildInput instanceInput = {
	    .type = OPTIX_BUILD_INPUT_TYPE_INSTANCES,
	    .instanceArray = {.instances = dInstances->getDeviceReadPtr(),
	                      .numInstances = static_cast<unsigned int>(dInstances->getCount())},
	};

	OptixAccelBuildOptions accelBuildOptions = {
	    .buildFlags =
	        OPTIX_BUILD_FLAG_ALLOW_UPDATE // TODO(prybicki): figure out if there's a faster way to update than the current one
	        | OPTIX_BUILD_FLAG_ALLOW_COMPACTION,
	    .operation = OPTIX_BUILD_OPERATION_BUILD};

	scratchpad.resizeToFit(instanceInput, accelBuildOptions);

	OptixAccelEmitDesc emitDesc = {
	    .result = scratchpad.dCompactedSize->getDeviceReadPtr(),
	    .type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
	};

	OptixTraversableHandle sceneHandle;
	CHECK_OPTIX(optixAccelBuild(Optix::getOrCreate().context, getStream()->getHandle(), &accelBuildOptions, &instanceInput, 1,
	                            scratchpad.dTemp->getDeviceReadPtr(),
	                            scratchpad.dTemp->getSizeOf() * scratchpad.dTemp->getCount(),
	                            scratchpad.dFull->getDeviceReadPtr(),
	                            scratchpad.dFull->getSizeOf() * scratchpad.dFull->getCount(), &sceneHandle, &emitDesc, 1));

	CHECK_CUDA(cudaStreamSynchronize(getStream()->getHandle()));

	// scratchpad.doCompaction(sceneHandle);

	return sceneHandle;
}

void Scene::requestASRebuild() { cachedAS.reset(); }

void Scene::requestSBTRebuild() { cachedSBT.reset(); }

CudaStream::Ptr Scene::getStream() const { return stream; }
