#pragma once

#include <set>
#include <string>
#include <optional>
#include <unordered_map>
#include <scene/ASBuildScratchpad.hpp>
#include <APIObject.hpp>

// TODO(prybicki): fix these includes
#include <data_types/ShaderBindingTableTypes.h>
#include <LaunchParams.h>

struct Entity;

/**
 * Class responsible for managing objects and meshes, building AS and SBT.
 * Ported from the PoC code which assumed 1:1 mesh:object relationship.
 * Because of that, when some objects share a mesh, SBT records will be duplicated.
 * This will still work, but it causes pointless CPU -> GPU copies.
 * TODO(prybicki): fix it
 */
struct Scene : APIObject<Scene>, std::enable_shared_from_this<Scene>
{
	static std::shared_ptr<Scene> defaultInstance();

	void addEntity(std::shared_ptr<Entity> entity);
	void removeEntity(std::shared_ptr<Entity> entity);

	std::size_t getObjectCount();

	OptixTraversableHandle getAS();
	OptixShaderBindingTable getSBT();

	void requestFullRebuild();
	void requestASRebuild();
	void requestSBTRebuild();

private:
	OptixShaderBindingTable buildSBT();
	OptixTraversableHandle buildAS();

private:
	std::set<std::shared_ptr<Entity>> entities;
	ASBuildScratchpad scratchpad;

	std::optional<OptixTraversableHandle> cachedAS;
	std::optional<OptixShaderBindingTable> cachedSBT;

	DeviceBuffer<OptixInstance> dInstances;
};

