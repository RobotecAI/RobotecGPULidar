#pragma once

#include <set>
#include <string>
#include <optional>
#include <scene/ASBuildScratchpad.hpp>
#include <APIObject.hpp>

// TODO(prybicki): fix these includes
#include <data_types/ShaderBindingTableTypes.h>
#include <LaunchParams.h>

struct SceneObject;

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

	void addObject(std::shared_ptr<SceneObject> object);

	std::size_t getObjectCount();
	std::shared_ptr<SceneObject> getObjectByName(std::string name);

	void removeObjectByName(std::string name);
	void removeAllObjects();

	OptixTraversableHandle getAS();
	OptixShaderBindingTable getSBT();

	void requestFullRebuild();
	void requestASRebuild();
	void requestSBTRebuild();

private:
	OptixShaderBindingTable buildSBT();
	OptixTraversableHandle buildAS();

private:
	std::unordered_map<std::string, std::shared_ptr<SceneObject>> objects;
	ASBuildScratchpad scratchpad;

	std::optional<OptixTraversableHandle> cachedAS;
	std::optional<OptixShaderBindingTable> cachedSBT;

	DeviceBuffer<OptixInstance> dInstances;
};

