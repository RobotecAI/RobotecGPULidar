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

#include <optix_stubs.h>
#include <Optix.hpp>
#include <DeviceBuffer.hpp>
#include <memory>

/**
 * Helper class to manage buffers used for building acceleration (GAS, IAS) structures and perform their compaction.
 */
struct ASBuildScratchpad
{
	bool resizeToFit(OptixBuildInput input, OptixAccelBuildOptions options);
	void doCompaction(OptixTraversableHandle& handle);

private:
	DeviceBuffer<uint64_t> dCompactedSize;
	DeviceBuffer<std::byte> dTemp;
	DeviceBuffer<std::byte> dFull;
	DeviceBuffer<std::byte> dCompact;

	friend struct Mesh;
	friend struct Object;
	friend struct Scene;
};
