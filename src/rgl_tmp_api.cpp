#include "LidarRenderer.h"
#include "LidarContext.hpp"
#include "data_types/LidarNoiseParams.h"

using namespace gdt;
using namespace fmt;

LidarRenderer* getWorkaround();

extern "C" {

void rgl_tmp_add_object(char* id, int vertex_count, vec3f* vertices, int index_count, vec3i* indices)
{
    auto transform = TransformMatrix::identity();
    getWorkaround()->addMeshRaw(id, vertex_count, vertices, index_count, indices, 12, transform.matrix_flat);
}

void rgl_tmp_update_object(char* id, float* transform)
{
    getWorkaround()->updateMeshTransformRawTmp(id, transform, 12);
}

void rgl_tmp_remove_object(char* id)
{
    getWorkaround()->removeMesh(id);
}

void rgl_tmp_remove_all_object(char* id)
{
    getWorkaround()->softReset();
}

void rgl_tmp_ctx_set_gaussian_noise_params(LidarContext* ctx, int angularNoiseType, float angularNoiseStDev, float angularNoiseMean,
    float distanceNoiseStDevBase, float distanceNoiseStDevRisePerMeter, float distanceNoiseMean)
{
    ctx->lidarNoiseParams = {.angularNoiseType = (AngularNoiseType)angularNoiseType,
                             .angularNoiseStDev = angularNoiseStDev,
                             .angularNoiseMean = angularNoiseMean,
                             .distanceNoiseStDevBase = distanceNoiseStDevBase,
                             .distanceNoiseStDevRisePerMeter = distanceNoiseStDevRisePerMeter,
                             .distanceNoiseMean = distanceNoiseMean};
}

}