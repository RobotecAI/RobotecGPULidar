#pragma once

#include <optix_types.h>

// RAII object to (de)initialize OptiX
struct Optix
{
	static Optix& instance();

	Optix();
	~Optix();

	OptixDeviceContext context = nullptr;
	OptixModule module = nullptr;
	OptixPipeline pipeline = nullptr;
	OptixProgramGroup raygenPG = nullptr;
	OptixProgramGroup missPG = nullptr;
	OptixProgramGroup hitgroupPG = nullptr;

private:
	void initializeStaticOptixStructures();
};
