#include <scene/Scene.hpp>
#include <scene/SceneObject.hpp>
#include <scene/Mesh.hpp>

#include "LidarContext.hpp"
#include "data_types/LidarNoiseParams.h"


using namespace gdt;
using namespace fmt;

extern "C" {

void rgl_tmp_add_object(char* id, int vertex_count, vec3f* vertices, int index_count, vec3i* indices)
{
	// For now, I'm preserving 1:1 Mesh <-> Object relation (to reduce the scope of the refactor)
	// In the future, it will be a high-priority optimization
	auto mesh = Mesh::create(vertex_count, reinterpret_cast<Vec3f*>(vertices),
	                         index_count, reinterpret_cast<Vec3i*>(indices)
	);

	auto object = SceneObject::create(mesh, std::string(id));
	Scene::defaultInstance()->addObject(object);
}

void rgl_tmp_update_object(char* id, float* transform)
{
	TransformMatrix matrix = TransformMatrix::fromPointer(transform);
	Scene::defaultInstance()->getObjectByName(id)->setTransform(matrix);
}

void rgl_tmp_remove_object(char* id)
{
	Scene::defaultInstance()->removeObjectByName(id);
}

void rgl_tmp_remove_all_object(char* id)
{
	Scene::defaultInstance()->removeAllObjects();
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