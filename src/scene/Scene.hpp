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
#include <mutex>
#include <string>
#include <optional>
#include <unordered_map>
#include <scene/ASBuildScratchpad.hpp>
#include <APIObject.hpp>

#include <Time.hpp>
#include <gpu/ShaderBindingTableTypes.h>
#include <memory/Array.hpp>

struct Entity;

/**
 * Class responsible for managing objects and meshes, building AS and SBT.
 * Ported from the PoC code which assumed 1:1 mesh:object relationship.
 * Because of that, when some objects share a mesh, SBT records will be duplicated.
 * This will still work, but it causes pointless CPU -> GPU copies.
 * TODO(prybicki): fix it
 *
 * This class may be accessed from different threads:
 * - client's thread doing API calls, modifying scene
 * - graph execution threads, requesting AS and SBT from RaytraceNode
 * As of now, Scene is not thread-safe, i.e. it is meant to be accessed only from the client's thread.
 * Calls that modify the scene waits until all current graph threads finish (done in API calls).
 * The only case when graph thread accesses scene is getAS() and getSBT(), which are locked.
 *
 */
struct Scene
{
	static Scene& instance();

	Scene(const Scene&) = delete;
	Scene(Scene&&) = delete;
	Scene& operator=(const Scene&) = delete;
	Scene& operator=(Scene&&) = delete;

	void addEntity(std::shared_ptr<Entity> entity);
	void removeEntity(std::shared_ptr<Entity> entity);
	void clear();

	void setTime(Time time)
	{
		prevTime = this->time;
		this->time = time;
	}
	std::optional<Time> getTime() const { return time; }
	std::optional<Time> getPrevTime() const { return prevTime; }
	std::optional<Time> getDeltaTime() const { return prevTime.has_value() ? std::optional(*time - *prevTime) : std::nullopt; }

	std::size_t getObjectCount() const;

	CudaStream::Ptr getStream() const;

	OptixTraversableHandle getASLocked();
	OptixShaderBindingTable getSBTLocked();

	void requestASRebuild();
	void requestSBTRebuild();

private:
	Scene();

	OptixShaderBindingTable buildSBT();
	OptixTraversableHandle buildAS();

private:
	CudaStream::Ptr stream;
	std::set<std::shared_ptr<Entity>> entities;
	ASBuildScratchpad scratchpad;

	std::mutex optixStructsMutex;
	std::optional<OptixTraversableHandle> cachedAS;
	std::optional<OptixShaderBindingTable> cachedSBT;

	// TODO: allow non-heap creation;
	DeviceSyncArray<OptixInstance>::Ptr dInstances = DeviceSyncArray<OptixInstance>::create();

	std::optional<Time> time;
	std::optional<Time> prevTime;
};
