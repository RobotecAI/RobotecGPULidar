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

#pragma once

#include <set>
#include <string>
#include <optional>
#include <unordered_map>
#include <scene/ASBuildScratchpad.hpp>
#include <APIObject.hpp>

#include <Time.hpp>
#include <gpu/ShaderBindingTableTypes.h>

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
	void clear();

	void setTime(Time time) { this->time = time; }
	std::optional<Time> getTime() { return time; }

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

	// TODO: allow non-heap creation;
	DeviceSyncArray<OptixInstance>::Ptr dInstances = DeviceSyncArray<OptixInstance>::create();

	std::optional<Time> time;
};
