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
	void resizeToFit(OptixBuildInput input, OptixAccelBuildOptions options);
	void doCompaction(OptixTraversableHandle& handle);

private:
	DeviceBuffer<uint64_t> NAMED(dCompactedSize);
	DeviceBuffer<std::byte> NAMED(dTemp);
	DeviceBuffer<std::byte> NAMED(dFull);
	DeviceBuffer<std::byte> NAMED(dCompact);

	friend struct Mesh;
	friend struct Object;
	friend struct Scene;
};
