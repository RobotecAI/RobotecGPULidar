#include <cmath>

#include <rgl/api/experimental.h>
#include <rgl/api/e2e_extensions.h>

#include <scene/Scene.hpp>
#include <scene/Entity.hpp>
#include <scene/Mesh.hpp>

#include <Lidar.hpp>
#include "data_types/LidarNoiseParams.h"
#include <macros/visibility.h>

static rgl_status_t lastErrorCode = RGL_SUCCESS;
static std::optional<std::string> lastErrorString = std::nullopt;

static rgl_status_t setStatus(rgl_status_t error, std::optional<std::string> auxErrStr = std::nullopt)
{
	lastErrorString = auxErrStr;
	return lastErrorCode = error;
}

static bool isRGLHealthy()
{
	static std::set recoverableErrors = {RGL_INVALID_ARGUMENT};
	return (lastErrorCode == RGL_SUCCESS) || recoverableErrors.contains(lastErrorCode);
}

template<typename Fn>
static rgl_status_t rglSafeCall(Fn fn)
{
	if (!isRGLHealthy()) {
		return setStatus(RGL_INVALID_STATE);
	}
	try {
		std::invoke(fn);
	}
	catch (std::invalid_argument &e) {
		return setStatus(RGL_INVALID_ARGUMENT, e.what());
	}
	catch (std::exception &e) {
		return setStatus(RGL_INTERNAL_EXCEPTION, e.what());
	}
	return setStatus(RGL_SUCCESS);
}

#define CHECK_ARG(expr)                                                                          \
do if (!(expr)) {                                                                                \
    auto msg = fmt::format("RGL API Error: Invalid argument, condition unsatisfied: {}", #expr); \
    logError(msg);                                                                               \
    throw std::invalid_argument(msg);                                                            \
} while(0)

extern "C" {

RGL_API rgl_status_t
rgl_get_version_info(int *out_major, int *out_minor, int *out_patch)
{
	// Short API version history:
	// 0.1: PoC via dlopen
	// 0.2: added E2E PCL formats (12, 24, 48), ringIds, postRaycastTransform
	// 0.3: implemented async calls
	// 0.4: added HasMesh to allow optimization
	// 0.5: implemented scene management via native API
	// 0.6: added gaussian noise API
	// 0.7: pre-open-source API cleanup
	// 0.8: removed hash and suffix args - will be logged instead
	return rglSafeCall([&]() {
		CHECK_ARG(out_major != nullptr);
		CHECK_ARG(out_minor != nullptr);
		CHECK_ARG(out_patch != nullptr);
		*out_major = 0;
		*out_minor = 8;
		*out_patch = 0;
	});
}

RGL_API void
rgl_get_last_error_string(const char **out_error_string)
{
	if (lastErrorString.has_value()) {
		*out_error_string = lastErrorString->c_str();
		return;
	}
	switch (lastErrorCode) {
		case RGL_SUCCESS:
			*out_error_string = "operation successful";
			break;
		case RGL_INVALID_ARGUMENT:
			*out_error_string = "invalid argument";
			break;
		case RGL_INTERNAL_EXCEPTION:
			*out_error_string = "unhandled internal exception";
			break;
		case RGL_INVALID_STATE:
			*out_error_string = "invalid state - unrecoverable error occurred";
			break;
		case RGL_NOT_IMPLEMENTED:
			*out_error_string = "operation not (yet) implemented";
			break;
		default:
			*out_error_string = "???";
			break;
	}
}


RGL_API rgl_status_t
rgl_mesh_create(rgl_mesh_t *out_mesh, rgl_vec3f *vertices, int vertex_count, rgl_vec3i *indices, int index_count)
{
	return rglSafeCall([&]() {
		CHECK_ARG(out_mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		CHECK_ARG(indices != nullptr);
		CHECK_ARG(index_count > 0);
		*out_mesh = Mesh::create(reinterpret_cast<Vec3f *>(vertices),
		                         vertex_count,
		                         reinterpret_cast<Vec3i *>(indices),
		                         index_count).get();
	});
}

RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh)
{
	return rglSafeCall([&]() {
		CHECK_ARG(mesh != nullptr);
		Mesh::release(mesh);
	});
}

RGL_API rgl_status_t
rgl_mesh_set_vertices(rgl_mesh_t mesh,
                      rgl_vec3f *vertices,
                      int vertex_count)
{
	return rglSafeCall([&]() {
		CHECK_ARG(mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		Mesh::validatePtr(mesh)->setVertices(reinterpret_cast<Vec3f *>(vertices), vertex_count);
	});
}

RGL_API rgl_status_t
rgl_entity_create(rgl_entity_t *out_entity, rgl_scene_t scene, rgl_mesh_t mesh)
{
	return rglSafeCall([&]() {
		CHECK_ARG(out_entity != nullptr);
		CHECK_ARG(mesh != nullptr);
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		*out_entity = Entity::create(Mesh::validatePtr(mesh)).get();
		Scene::validatePtr(scene)->addEntity(Entity::validatePtr(*out_entity));
	});
}

RGL_API rgl_status_t
rgl_entity_destroy(rgl_entity_t entity)
{
	return rglSafeCall([&]() {
		CHECK_ARG(entity != nullptr);
		auto entitySafe = Entity::validatePtr(entity);
		if (auto sceneShared = entitySafe->scene.lock()) {
			sceneShared->removeEntity(entitySafe);
			Entity::release(entity);
		} else {
			throw std::logic_error("Entity's scene does not exist");
		}
	});
}

RGL_API rgl_status_t
rgl_entity_set_pose(rgl_entity_t entity, rgl_mat3x4f *local_to_world_tf)
{
	return rglSafeCall([&]() {
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(local_to_world_tf != nullptr);
		auto tf = TransformMatrix::fromPointer(reinterpret_cast<float *>(&local_to_world_tf->value[0][0]));
		Entity::validatePtr(entity)->setTransform(tf);
	});
}

RGL_API rgl_status_t
rgl_lidar_create(rgl_lidar_t *out_lidar,
                 rgl_mat3x4f *ray_transforms,
                 int ray_transforms_count)
{
	return rglSafeCall([&]() {
		CHECK_ARG(out_lidar != nullptr);
		CHECK_ARG(ray_transforms != nullptr);
		CHECK_ARG(ray_transforms_count > 0);
		*out_lidar = Lidar::create(reinterpret_cast<TransformMatrix *>(ray_transforms), ray_transforms_count).get();
	});
}

RGL_API rgl_status_t
rgl_lidar_create_with_frames(rgl_lidar_t* out_lidar, rgl_mat3x4f *ray_transforms, int ray_transforms_count, int* frameSizes, int frameCount)
{
    return rglSafeCall([&]() {
        CHECK_ARG(out_lidar != nullptr);
        CHECK_ARG(ray_transforms != nullptr);
        CHECK_ARG(ray_transforms_count > 0);
        // CHECK_ARG(frameCount > 1);
        *out_lidar = Lidar::create(reinterpret_cast<TransformMatrix *>(ray_transforms), ray_transforms_count, frameSizes, frameCount).get();
    });
}

RGL_API rgl_status_t
rgl_lidar_next_frame(rgl_lidar_t lidar)
{
    return rglSafeCall([&]() {
        CHECK_ARG(lidar != nullptr);
        Lidar::validatePtr(lidar)->nextFrame();
    });
}

RGL_API rgl_status_t
rgl_lidar_set_range(rgl_lidar_t lidar, float range)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(!std::isnan(range));
		Lidar::validatePtr(lidar)->range = range;
	});
}

RGL_API rgl_status_t
rgl_lidar_destroy(rgl_lidar_t lidar)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		Lidar::release(lidar);
	});
}

RGL_API rgl_status_t
rgl_lidar_set_pose(rgl_lidar_t lidar, rgl_mat3x4f *local_to_world_tf)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(local_to_world_tf != nullptr);
		Lidar::validatePtr(lidar)->lidarPose = TransformMatrix::fromPointer(&local_to_world_tf->value[0][0]);
	});
}

RGL_API rgl_status_t
rgl_lidar_raytrace_async(rgl_scene_t scene, rgl_lidar_t lidar)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		Lidar::validatePtr(lidar)->scheduleRaycast(Scene::validatePtr(scene));
	});
}

RGL_API rgl_status_t
rgl_lidar_get_output_size(rgl_lidar_t lidar, int *out_size)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(out_size != nullptr);
		*out_size = Lidar::validatePtr(lidar)->getResultsSize();
	});
}

RGL_API rgl_status_t
rgl_lidar_get_output_data(rgl_lidar_t lidar, rgl_format_t format, void *out_data)
{
	return rglSafeCall([&]() {
		int fmtVal = static_cast<int>(format);
		bool formatOK = (RGL_FORMAT_INVALID < fmtVal && fmtVal < RGL_FORMAT_COUNT);
		bool formatE2EOK = (RGL_FORMAT_E2E_INVALID < fmtVal && fmtVal < RGL_FORMAT_E2E_COUNT);
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(formatOK || formatE2EOK);
		CHECK_ARG(out_data != nullptr);
		Lidar::validatePtr(lidar)->getResults(format, out_data);
	});
}

RGL_API rgl_status_t
rgl_lidar_set_ring_indices(rgl_lidar_t lidar, int *ring_ids, int ring_ids_count)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(ring_ids != nullptr);
		CHECK_ARG(ring_ids_count > 0);
		Lidar::validatePtr(lidar)->setRingIds(ring_ids, ring_ids_count);
	});
}

RGL_API rgl_status_t
rgl_lidar_set_gaussian_noise_params(rgl_lidar_t lidar,
                                    int angular_noise_type,
                                    float angular_noise_stddev,
                                    float angular_noise_mean,
                                    float distance_noise_stddev_base,
                                    float distance_noise_stddev_rise_per_meter,
                                    float distance_noise_mean)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(angular_noise_type == RAY_BASED || angular_noise_type == HITPOINT_BASED);
		CHECK_ARG(angular_noise_stddev >= 0.0f);
		CHECK_ARG(distance_noise_stddev_base >= 0.0f);
		CHECK_ARG(distance_noise_stddev_rise_per_meter >= 0.0f);
		Lidar::validatePtr(lidar)->lidarNoiseParams = {
			.angularNoiseType = (AngularNoiseType) angular_noise_type,
			.angularNoiseStDev = angular_noise_stddev,
			.angularNoiseMean = angular_noise_mean,
			.distanceNoiseStDevBase = distance_noise_stddev_base,
			.distanceNoiseStDevRisePerMeter = distance_noise_stddev_rise_per_meter,
			.distanceNoiseMean = distance_noise_mean
		};
	});
}

RGL_API rgl_status_t
rgl_lidar_set_post_raycast_transform(rgl_lidar_t lidar, rgl_mat3x4f *transform)
{
	return rglSafeCall([&]() {
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(transform != nullptr);
		Lidar::validatePtr(lidar)->rosTransform = TransformMatrix::fromPointer(&transform->value[0][0]);
	});
}

}