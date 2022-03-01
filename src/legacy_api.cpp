#include <memory>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <vector>

#include <fmt/color.h>

#include "LidarRenderer.h"
#include "LidarContext.hpp"
#include "data_types/PointTypes.h"
#include "visibility_control.h"

using namespace gdt;
using namespace fmt;
static std::string last_gpu_library_error = ""; // no support for multithreading

void br() {}

#define LIDAR_GPU_TRY_CATCH(call) \
    do                                                           \
    {                                                            \
        try {                                                    \
            call;                                                \
        } catch (std::runtime_error & err) {                     \
            last_gpu_library_error = err.what();                 \
            fprintf(stderr, "Runtime exception %s", err.what()); \
            return GPULIDAR_ERROR;                               \
        }                                                        \
    } while(0)



LidarRenderer* getWorkaround() {
    static std::unique_ptr<LidarRenderer> workaround = std::make_unique<LidarRenderer>();
    return workaround.get();
}

extern "C" {

// First attempt - on any error the module is deemed unusable
enum GPULIDAR_RETURN_CODE {
    ROBOTEC_SUCCESS = 0,
    GPULIDAR_ERROR
};

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_HelloWorld()
{
    fmt::print(stderr, "Hello world unity @@");
    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_CreateNativeRaycaster(LidarRenderer** lidarRenderer)
{
    LIDAR_GPU_TRY_CATCH(*lidarRenderer = getWorkaround());
    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
void Internal_DestroyNativeRaycaster(LidarRenderer* lidarRenderer)
{
    // OK, Boomer.
    lidarRenderer->softReset();
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
    LIDAR_GPU_TRY_CATCH(lidarRenderer->addMeshRaw(mesh_id, mesh_size, vertices, normals, texture_coordinates,
                                                  indices_size, indices, transform_size, transform));
    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_HasMesh(LidarRenderer* lidarRenderer, char* mesh_id, bool* outHasMesh)
{
    LIDAR_GPU_TRY_CATCH(*outHasMesh = lidarRenderer->hasMesh(mesh_id));
    return ROBOTEC_SUCCESS;
}


GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_RemoveMesh(LidarRenderer* lidarRenderer, char* mesh_id)
{
    LIDAR_GPU_TRY_CATCH(lidarRenderer->removeMeshRawTmp(mesh_id));
    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_UpdateMeshTransform(LidarRenderer* lidarRenderer, char* id, float* transform, int transform_size)
{
    LIDAR_GPU_TRY_CATCH(lidarRenderer->updateMeshTransformRawTmp(id, transform, transform_size));
    return ROBOTEC_SUCCESS;
}


GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_CreateLidarContext(LidarRenderer* lidarRenderer, void** outLidarCtx, float* rayPosesFloats, size_t rayPosesFloatCount, int* lidarArrayRingIds, size_t lidarArrayRingCount)
{
    auto* rayPosesTyped = reinterpret_cast<TransformMatrix*>(rayPosesFloats);
    auto rayPosesCount = static_cast<int>(sizeof(float) * rayPosesFloatCount / sizeof(*rayPosesTyped));
    LIDAR_GPU_TRY_CATCH(*outLidarCtx = new LidarContext(rayPosesTyped, rayPosesCount, lidarArrayRingIds, lidarArrayRingCount));
    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_DestroyLidarContext(LidarRenderer* lidarRenderer, LidarContext* lidarCtx)
{
    LIDAR_GPU_TRY_CATCH(
        if (lidarCtx == nullptr) {
            throw std::invalid_argument("lidarCtx == null");
        }
    );
    LIDAR_GPU_TRY_CATCH(delete lidarCtx);
    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_Raycast(LidarRenderer* lidarRenderer, LidarContext* lidarCtx, char* source_id, float* lidarPose, float* rosTransform, float range)
{
    auto* lidarPoseTyped = reinterpret_cast<TransformMatrix*>(lidarPose);
    auto* rosTransformTyped = reinterpret_cast<TransformMatrix*>(rosTransform);

    LIDAR_GPU_TRY_CATCH(
        if (lidarCtx == nullptr) {
            throw std::invalid_argument("lidarCtx == null");
        }
    );

    LIDAR_GPU_TRY_CATCH(lidarRenderer->renderCtx(lidarCtx,
                                              *lidarPoseTyped,
                                              *rosTransformTyped,
                                              range));

    return ROBOTEC_SUCCESS;
}

GPU_LIDAR_RAYCASTER_C_EXPORT
int Internal_GetPoints(LidarRenderer* lidarRenderer, LidarContext* lidarCtx, void* xyz, void* pcl12, void* pcl24, void* pcl48, int* results_count)
{
    // Once upon a time there was an urgent request to implement PCL generation on the GPU.
    // To remove redundant copies, the caller will provide a pointer to store the data.
    // To provide such a pointer, it needs to know the size of the PCL upfront.
    // Adding a new call turned out to be problematic (not working out-of-the-box).
    // Therefore, GetPoints, if provided a negative results_count,
    // it will be filled with the number of points in the result PCL.
    // No other action will be performed.
    if (*results_count < 0) {
        LIDAR_GPU_TRY_CATCH(*results_count = lidarRenderer->getResultPointCloudSizeCtx(lidarCtx));
        return ROBOTEC_SUCCESS;
    }

    LIDAR_GPU_TRY_CATCH(
        if (lidarCtx == nullptr) {
            throw std::invalid_argument("lidarCtx == null");
        }
    );


    LIDAR_GPU_TRY_CATCH(lidarRenderer->downloadPointsCtx(
        lidarCtx,
        *results_count,
        reinterpret_cast<Point3f*>(xyz),
        reinterpret_cast<PCL12*>(pcl12),
        reinterpret_cast<PCL24*>(pcl24),
        reinterpret_cast<PCL48*>(pcl48))
    );

    return ROBOTEC_SUCCESS;
}

} // extern "C"
