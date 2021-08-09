#include <memory>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <vector>

#include <fmt/color.h>

#include "LidarSource.h"
#include "LidarRenderer.h"
#include "PointTypes.h"
#include "RaycastResult.h"
#include "visibility_control.h"

using namespace gdt;
using namespace fmt;
static std::string last_gpu_library_error = ""; // no support for multithreading

#define LIDAR_GPU_TRY_CATCH(call)                                \
    {                                                            \
        try {                                                    \
            call;                                                \
        } catch (std::runtime_error & err) {                     \
            last_gpu_library_error = err.what();                 \
            fprintf(stderr, "Runtime exception %s", err.what()); \
            return GPULIDAR_ERROR;                               \
        }                                                        \
    }

extern "C" {

// First attempt - on any error the module is deemed unusable
enum GPULIDAR_RETURN_CODE {
    GPULIDAR_SUCCESS = 0,
    GPULIDAR_ERROR
};

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_CreateNativeRaycaster(LidarRenderer** lidarRenderer)
{
    LIDAR_GPU_TRY_CATCH(*lidarRenderer = new LidarRenderer());
    return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_DestroyNativeRaycaster(LidarRenderer* lidarRenderer)
{
    delete lidarRenderer;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
const char* Internal_GetLastError()
{
    // Return pointer to memory of last_gpu_library_error. Interpreted as null-terminated string
    return last_gpu_library_error.c_str();
}

// TODO - optimize this POC
GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_AddMesh(LidarRenderer* lidarRenderer, char* mesh_id, float* transform, bool is_global, vec3f* vertices, vec3f* normals,
    vec2f* texture_coordinates, vec3i* indices, int indices_size, int mesh_size, int transform_size)
{
    (void) is_global;
    LIDAR_GPU_TRY_CATCH(lidarRenderer->addMeshRawTmp(mesh_id, mesh_size, vertices, normals, texture_coordinates,
                                                          indices_size, indices, transform_size, transform));
    return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_RemoveMesh(LidarRenderer* lidarRenderer, char* mesh_id)
{
    LIDAR_GPU_TRY_CATCH(lidarRenderer->removeMeshRawTmp(mesh_id));
    return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_UpdateMeshTransform(LidarRenderer* lidarRenderer, char* id, float* transform, int transform_size)
{
    LIDAR_GPU_TRY_CATCH(lidarRenderer->updateMeshTransformRawTmp(id, transform, transform_size));
    return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_Raycast(LidarRenderer* lidarRenderer, char* source_id, Point source_pos, Point* directions, int directions_count, float range)
{
    LidarSource source {source_id, source_pos, range, directions_count, directions};

    LIDAR_GPU_TRY_CATCH(lidarRenderer->resize({source})); // TODO - resize is not usually required if lidars don't change (?)
    LIDAR_GPU_TRY_CATCH(lidarRenderer->render({source})); // TODO - support more sources when needed in the higher interface

    return GPULIDAR_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_GetPoints(LidarRenderer* lidarRenderer, const LidarPoint** results, int* results_count)
{
    const RaycastResults* allLidarsResults = nullptr;

    LIDAR_GPU_TRY_CATCH(allLidarsResults = lidarRenderer->downloadPoints());

    if (allLidarsResults == nullptr || allLidarsResults->size() == 0) {
        results_count = nullptr;
        return GPULIDAR_SUCCESS;
    }

    *results_count = allLidarsResults->at(0).points.size();
    *results = reinterpret_cast<const LidarPoint*>(allLidarsResults->at(0).points.data());
    return GPULIDAR_SUCCESS;
}

} // extern "C"
