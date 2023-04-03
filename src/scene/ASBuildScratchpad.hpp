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

#include <memory>

#include <optix_stubs.h>

#include <Optix.hpp>
#include <memory/DeviceSyncArray.hpp>
#include <memory/HostPinnedArray.hpp>

/**
 * Helper class to manage buffers used for building acceleration (GAS, IAS) structures and perform their compaction.
 */
struct ASBuildScratchpad
{
	void resizeToFit(OptixBuildInput input, OptixAccelBuildOptions options);
	void doCompaction(OptixTraversableHandle& handle);

private:
	HostPinnedArray<uint64_t>::Ptr hCompactedSize = HostPinnedArray<uint64_t>::create();
	DeviceSyncArray<uint64_t>::Ptr dCompactedSize = DeviceSyncArray<uint64_t>::create();
	DeviceSyncArray<std::byte>::Ptr dTemp = DeviceSyncArray<std::byte>::create();
	DeviceSyncArray<std::byte>::Ptr dFull = DeviceSyncArray<std::byte>::create();
	DeviceSyncArray<std::byte>::Ptr dCompact = DeviceSyncArray<std::byte>::create();

	friend struct Mesh;
	friend struct Scene;
};
