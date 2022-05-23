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

void Scene::addEntity(std::shared_ptr<Entity> entity)
{
	// TODO: remove this limitation
	for (auto&& e : entities) {
		if (e->mesh.use_count() > 2) { // APIObject<Mesh>::instance + Entity::mesh
			auto msg = "Entities sharing mesh is not yet implemented! Please use separate mesh instances for each entity";
			throw std::logic_error(msg);
		}
	}
	entity->scene = weak_from_this();
	entities.insert(entity);
	requestFullRebuild();
}

void Scene::removeEntity(std::shared_ptr<Entity> entity)
{
	entities.erase(entity);
}

// std::shared_ptr<Entity> Scene::getObjectByName(std::string name)
// {
// 	return objects.at(name);
// }

// void Scene::removeObjectByName(std::string name)
// {
// 	logInfo("[RGL] Removing object name={}", name);
// 	auto object = objects.at(name);
// 	Entity::release(object);
// 	objects.erase(name);
// 	logWarn("{}\n", Entity::instances.size());
// 	requestFullRebuild();
// }

// void Scene::removeAllEntities()
// {
// 	for (auto&& [name, object] : entities) {
// 		Entity::release(object.get());
// 	}
// 	objects.clear();
// 	requestFullRebuild();
// }

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
	logInfo("[RGL] buildSBT");
	static DeviceBuffer<HitgroupRecord> dHitgroupRecords("hitgroupRecord");
	static DeviceBuffer<RaygenRecord> dRaygenRecords("raygenRecord");
	static DeviceBuffer<MissRecord> dMissRecords("missRecord");

	logInfo("[RGL] Building SBT using {} objects\n", entities.size());
	std::vector<HitgroupRecord> hHitgroupRecords;
	for (auto&& entity : entities) {
		auto mesh = entity->mesh;

		hHitgroupRecords.emplace_back(); // TODO(prybicki): fix, this is weird
		HitgroupRecord *hr = &(*hHitgroupRecords.rbegin());
		OPTIX_CHECK(optixSbtRecordPackHeader(Optix::instance().hitgroupPG, hr));
		hr->data = TriangleMeshSBTData{
			.vertex = mesh->dVertices.readDevice(),
			.index = mesh->dIndices.readDevice(),
			.vertex_count = mesh->dVertices.getElemCount(),
			.index_count = mesh->dIndices.getElemCount(),
		};
	}
	dHitgroupRecords.copyFromHost(hHitgroupRecords);

	RaygenRecord hRaygenRecord;
	OPTIX_CHECK(optixSbtRecordPackHeader(Optix::instance().raygenPG, &hRaygenRecord));
	dRaygenRecords.copyFromHost(&hRaygenRecord, 1);

	MissRecord hMissRecord;
	OPTIX_CHECK(optixSbtRecordPackHeader(Optix::instance().missPG, &hMissRecord));
	dMissRecords.copyFromHost(&hMissRecord, 1);

	return OptixShaderBindingTable{
		.raygenRecord = dRaygenRecords.readDeviceRaw(),
		.missRecordBase = dMissRecords.readDeviceRaw(),
		.missRecordStrideInBytes = sizeof(MissRecord),
		.missRecordCount = 1U,
		.hitgroupRecordBase = dHitgroupRecords.readDeviceRaw(),
		.hitgroupRecordStrideInBytes = sizeof(HitgroupRecord),
		.hitgroupRecordCount = static_cast<unsigned>(dHitgroupRecords.getElemCount()),
	};
}

OptixTraversableHandle Scene::buildAS()
{
	logInfo("[RGL] buildAS\n");
	std::vector<OptixInstance> instances;
	for (auto&& entity : entities) {
		// TODO(prybicki): this is somewhat inefficient, because most of the time only transform changes.
		instances.push_back(entity->getIAS(static_cast<int>(instances.size())));
	}

	// *** *** *** ACHTUNG *** *** ***
	// Calls to cudaMemcpy below are a duck-tape for synchronizing all streams from LidarContexts.
	dInstances.resizeToFit(instances.size());
	dInstances.copyFromHost(instances);

	OptixBuildInput instanceInput = {
	.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES,
	.instanceArray = {
	.instances = dInstances.readDeviceRaw(),
	.numInstances = static_cast<unsigned int>(dInstances.getElemCount())
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
	.result = scratchpad.dCompactedSize.readDeviceRaw(),
	.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE,
	};

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

	CHECK_CUDA(cudaStreamSynchronize(nullptr));
	scratchpad.doCompaction(sceneHandle);

	// TODO(prybicki): use a non-null stream
	CHECK_CUDA(cudaStreamSynchronize(nullptr));

	logInfo("[RGL] AS built\n");
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
