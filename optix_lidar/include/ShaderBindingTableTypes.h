#pragma once

__align__(OPTIX_SBT_RECORD_ALIGNMENT)
struct RaygenRecord
{
	char header[OPTIX_SBT_RECORD_HEADER_SIZE];
};

__align__(OPTIX_SBT_RECORD_ALIGNMENT)
struct MissRecord
{
	char header[OPTIX_SBT_RECORD_HEADER_SIZE];
};

__align__(OPTIX_SBT_RECORD_ALIGNMENT)
struct HitgroupRecord
{
	char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	TriangleMeshSBTData data;
};
