#include <scene/Scene.hpp>
#include <scene/SceneObject.hpp>
#include <scene/Mesh.hpp>

#include "LidarContext.hpp"
#include "LidarRenderer.h"
#include "data_types/LidarNoiseParams.h"
#include <macros/visibility.hpp>

// TODO: proper naming scheme:
// exp_rgl_ctx_raycast_async

// TODO: exception handling

using namespace gdt;
using namespace fmt;

extern "C" {

RGL_API void rgl_exp_add_object(char* id, int vertex_count, vec3f* vertices, int index_count, vec3i* indices)
{
	// For now, I'm preserving 1:1 Mesh <-> Object relation (to reduce the scope of the refactor)
	// In the future, it will be a high-priority optimization
	auto mesh = Mesh::create(vertex_count, reinterpret_cast<Vec3f*>(vertices),
	                         index_count, reinterpret_cast<Vec3i*>(indices)
	);

	auto object = SceneObject::create(mesh, std::string(id));
	Scene::defaultInstance()->addObject(object);
}

RGL_API void rgl_exp_update_object_transform(char* id, float* transform)
{
	TransformMatrix matrix = TransformMatrix::fromPointer(transform);
	Scene::defaultInstance()->getObjectByName(id)->setTransform(matrix);
}

RGL_API void rgl_exp_update_object_vertices(char* id, int vertex_count, Vec3f* vertices)
{
    Scene::defaultInstance()->getObjectByName(id)->mesh->setVertices(vertex_count, vertices);

    // TODO: figure out a way to do this automagically
    // NOTE: weak_from_this in Mesh constructor is not an option :(
    Scene::defaultInstance()->requestSBTRebuild();
    Scene::defaultInstance()->requestASRebuild();
}

RGL_API void rgl_exp_remove_object(char* id)
{
	Scene::defaultInstance()->removeObjectByName(id);
}

RGL_API void rgl_exp_remove_all_object(char* id)
{
	Scene::defaultInstance()->removeAllObjects();
}

RGL_API void rgl_exp_ctx_set_gaussian_noise_params(LidarContext* ctx, int angularNoiseType, float angularNoiseStDev, float angularNoiseMean,
    float distanceNoiseStDevBase, float distanceNoiseStDevRisePerMeter, float distanceNoiseMean)
{
    ctx->lidarNoiseParams = {.angularNoiseType = (AngularNoiseType)angularNoiseType,
                             .angularNoiseStDev = angularNoiseStDev,
                             .angularNoiseMean = angularNoiseMean,
                             .distanceNoiseStDevBase = distanceNoiseStDevBase,
                             .distanceNoiseStDevRisePerMeter = distanceNoiseStDevRisePerMeter,
                             .distanceNoiseMean = distanceNoiseMean};
}

// TODO change types float -> TransformMatrix
RGL_API void rgl_exp_create_lidar_ctx(void** outLidarCtx, float* rayPosesFloats, int rayPosesFloatCount, int* lidarArrayRingIds, int lidarArrayRingCount)
{
	auto* rayPosesTyped = reinterpret_cast<TransformMatrix*>(rayPosesFloats);
	auto rayPosesCount = static_cast<int>(sizeof(float) * rayPosesFloatCount / sizeof(*rayPosesTyped));
	*outLidarCtx = new LidarContext(rayPosesTyped, rayPosesCount, lidarArrayRingIds, lidarArrayRingCount);
}

RGL_API void rgl_exp_destroy_lidar_ctx(LidarContext* lidarCtx)
{
	if (lidarCtx == nullptr) {
		std::abort(); // TODO(prybicki): proper handling
		//throw std::invalid_argument("lidarCtx == null");
	}
	delete lidarCtx;
}

// TODO change type lidarPose, rosTransform -> TransformMatrix
RGL_API void rgl_exp_raycast_async(LidarContext* lidarCtx, float* lidarPose, float* rosTransform, float range)
{
	auto* lidarPoseTyped = reinterpret_cast<TransformMatrix*>(lidarPose);
	auto* rosTransformTyped = reinterpret_cast<TransformMatrix*>(rosTransform);

	if (lidarCtx == nullptr) {
		std::abort(); // TODO handle
		// throw std::invalid_argument("lidarCtx == null");
	}

	LidarRenderer::instance().renderCtx(lidarCtx, *lidarPoseTyped, *rosTransformTyped, range);
}

RGL_API void rgl_exp_get_points(LidarContext* lidarCtx, void* xyz, void* pcl12, void* pcl24, void* pcl48, int* results_count)
{
	// Buffers passed via pointers must be large enough to contain PCL if all points are hitpoints.
	*results_count = LidarRenderer::instance().getResultPointCloudSizeCtx(lidarCtx);

	if (lidarCtx == nullptr) {
		// throw std::invalid_argument("lidarCtx == null");
		// TODO handle
		std::abort();
	}


	LidarRenderer::instance().downloadPointsCtx(
		lidarCtx,
		*results_count,
		reinterpret_cast<Point3f*>(xyz),
		reinterpret_cast<PCL12*>(pcl12),
		reinterpret_cast<PCL24*>(pcl24),
		reinterpret_cast<PCL48*>(pcl48)
	);
}

}