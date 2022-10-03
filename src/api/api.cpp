#include <cmath>
#include <spdlog/common.h>

#include <rgl/api/experimental.h>
#include <rgl/api/e2e_extensions.h>

#include <scene/Scene.hpp>
#include <scene/Entity.hpp>
#include <scene/Mesh.hpp>

#include <Lidar.hpp>
#include <Tape.h>
#include <RGLExceptions.hpp>
#include <macros/visibility.h>

#include <repr.hpp>

static rgl_status_t lastStatusCode = RGL_SUCCESS;
static std::optional<std::string> lastStatusString = std::nullopt;

static bool canContinueAfterStatus(rgl_status_t status)
{
	// Set constructor may throw, hence lazy initialization.
	static std::set recoverableErrors = {
		RGL_INVALID_ARGUMENT,
		RGL_INVALID_API_OBJECT,
		RGL_INVALID_FILE_PATH,
		RGL_NOT_IMPLEMENTED,
		RGL_TAPE_ERROR
	};
	return status == RGL_SUCCESS || recoverableErrors.contains(status);
};

static rgl_status_t updateAPIState(rgl_status_t status, std::optional<std::string> auxErrStr = std::nullopt)
{
	lastStatusString = auxErrStr;
	lastStatusCode = status;
	if (status != RGL_SUCCESS) {
		// Report API error in log file
		const char* msg = nullptr;
		rgl_get_last_error_string(&msg);
		if (canContinueAfterStatus(lastStatusCode)) {
			RGL_ERROR("Recoverable error (code={}): {}", lastStatusCode, msg);
		} else {
			RGL_CRITICAL("Unrecoverable error (code={}): {}", lastStatusCode, msg);
		}
		Logger::instance().flush();
	}
	return status;
}

template<typename Fn>
static rgl_status_t rglSafeCall(Fn fn)
{
	if (!canContinueAfterStatus(lastStatusCode)) {
		if (lastStatusCode != RGL_LOGGING_ERROR) {
			RGL_CRITICAL("Logging disabled due to the previous fatal error");
			try {
				Logger::instance().configure(RGL_LOG_LEVEL_OFF, std::nullopt, false);
			}
			catch (std::exception& e) {}
		}
		return updateAPIState(RGL_INVALID_STATE);
	}
	try {
		std::invoke(fn);
	}
	catch (spdlog::spdlog_ex& e) {
		return updateAPIState(RGL_LOGGING_ERROR, e.what());
	}
	catch (InvalidAPIObject& e) {
		return updateAPIState(RGL_INVALID_API_OBJECT, e.what());
	}
	catch (InvalidFilePath& e) {
		return updateAPIState(RGL_INVALID_FILE_PATH, e.what());
	}
	catch (std::invalid_argument& e) {
		return updateAPIState(RGL_INVALID_ARGUMENT, e.what());
	}
	catch (RecordError& e) {
		return updateAPIState(RGL_TAPE_ERROR, e.what());
	}
	catch (std::exception& e) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, e.what());
	}
	return updateAPIState(RGL_SUCCESS);
}

#define CHECK_ARG(expr)                                                                          \
do if (!(expr)) {                                                                                \
    auto msg = fmt::format("RGL API Error: Invalid argument, condition unsatisfied: {}", #expr); \
    throw std::invalid_argument(msg);                                                            \
} while(0)

extern "C" {

RGL_API rgl_status_t
rgl_get_version_info(int* out_major, int* out_minor, int* out_patch)
{
	// Short API version history:
	// This is a brief shortcut for devs!
	// 0.1: PoC via dlopen
	// 0.2: added E2E PCL formats (12, 24, 48), ringIds, postRaycastTransform
	// 0.3: implemented async calls
	// 0.4: added HasMesh to allow optimization
	// 0.5: implemented scene management via native API
	// 0.6: added gaussian noise API
	// 0.7: pre-open-source API cleanup
	// 0.8: removed hash and suffix args - will be logged instead
	// 0.9: added rgl_configure_logging
	// 0.9.1: optimized rgl_mesh_update_vertices
	// 0.9.2: remove external dependency (gdt)
	// 0.9.3: add API unit tests with fixes, in particular: optix logging
	// 0.10.0: entities can now share meshes
	// 0.10.1: API const correctness, added INVALID_OBJECT error, minor internal changes
	// 0.10.2: Fixed Lidar::getResults writing too many bytes
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_get_version_info(out_major={}, out_minor={}, out_patch={})", (void*) out_major,
			  (void*) out_minor, (void*) out_patch);
		CHECK_ARG(out_major != nullptr);
		CHECK_ARG(out_minor != nullptr);
		CHECK_ARG(out_patch != nullptr);
		*out_major = RGL_VERSION_MAJOR;
		*out_minor = RGL_VERSION_MINOR;
		*out_patch = RGL_VERSION_PATCH;
	});
}

RGL_API rgl_status_t
rgl_configure_logging(rgl_log_level_t log_level, const char* log_file_path, bool use_stdout)
{
	return rglSafeCall([&]() {
		// No logging here, Logger::configure() will print logs after its initialization.
		CHECK_ARG(0 <= log_level && log_level <= 6);
		// Constructing string from nullptr is undefined behavior!
		auto logFilePath = log_file_path == nullptr ? std::nullopt : std::optional(log_file_path);
		Logger::instance().configure(log_level, logFilePath, use_stdout);
	});
}

RGL_API void
rgl_get_last_error_string(const char** out_error_string)
{
	// No logging here for now, since it may throw.
	if (lastStatusString.has_value()) {
		*out_error_string = lastStatusString->c_str();
		return;
	}
	switch (lastStatusCode) {
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
		case RGL_LOGGING_ERROR:
			*out_error_string = "spdlog error";
		case RGL_INVALID_FILE_PATH:
			*out_error_string = "invalid file path";
		case RGL_TAPE_ERROR:
			*out_error_string = "tape error";
		default:
			*out_error_string = "???";
			break;
	}
}

RGL_API rgl_status_t
rgl_cleanup(void)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_cleanup()");
		Entity::instances.clear();
		Mesh::instances.clear();
		Lidar::instances.clear();
		Scene::defaultInstance()->clear();
	});
}


RGL_API rgl_status_t
rgl_mesh_create(rgl_mesh_t* out_mesh, const rgl_vec3f* vertices, int vertex_count, const rgl_vec3i* indices,
		int index_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_mesh_create(out_mesh={}, vertices={}, indices={})",
			  (void*) out_mesh, repr(vertices, vertex_count), repr(indices, index_count, 1));
		CHECK_ARG(out_mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		CHECK_ARG(indices != nullptr);
		CHECK_ARG(index_count > 0);
		*out_mesh = Mesh::create(reinterpret_cast<const Vec3f*>(vertices),
					 vertex_count,
					 reinterpret_cast<const Vec3i*>(indices),
					 index_count).get();
		if (tapeRecord.has_value()) {
			tapeRecord->recordMeshCreate(out_mesh, vertices, vertex_count, indices, index_count);
		}
	});
}

RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_mesh_destroy(mesh={})", (void*) mesh);
		CHECK_ARG(mesh != nullptr);
		Mesh::release(mesh);
		if (tapeRecord.has_value()) {
			tapeRecord->recordMeshDestroy(mesh);
		}
	});
}

RGL_API rgl_status_t
rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertex_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_mesh_update_vertices(mesh={}, vertices={})", (void*) mesh, repr(vertices, vertex_count));
		CHECK_ARG(mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		Mesh::validatePtr(mesh)->updateVertices(reinterpret_cast<const Vec3f*>(vertices), vertex_count);
		if (tapeRecord.has_value()) {
			tapeRecord->recordMeshUpdateVertices(mesh, vertices, vertex_count);
		}
	});
}

RGL_API rgl_status_t
rgl_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_entity_create(out_entity={}, scene={}, mesh={})", (void*) out_entity, (void*) scene,
			  (void*) mesh);
		CHECK_ARG(out_entity != nullptr);
		CHECK_ARG(mesh != nullptr);
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		*out_entity = Entity::create(Mesh::validatePtr(mesh)).get();
		Scene::validatePtr(scene)->addEntity(Entity::validatePtr(*out_entity));
		if (tapeRecord.has_value()) {
			tapeRecord->recordEntityCreate(out_entity, scene, mesh);
		}
	});
}

RGL_API rgl_status_t
rgl_entity_destroy(rgl_entity_t entity)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_entity_destroy(entity={})", (void*) entity);
		CHECK_ARG(entity != nullptr);
		auto entitySafe = Entity::validatePtr(entity);
		if (auto sceneShared = entitySafe->scene.lock()) {
			sceneShared->removeEntity(entitySafe);
			Entity::release(entity);
		} else {
			throw std::logic_error("Entity's scene does not exist");
		}
		if (tapeRecord.has_value()) {
			tapeRecord->recordEntityDestroy(entity);
		}
	});
}

RGL_API rgl_status_t
rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_entity_set_pose(entity={}, local_to_world_tf={})", (void*) entity,
			  repr(local_to_world_tf, 1));
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(local_to_world_tf != nullptr);
		auto tf = Mat3x4f::fromRaw(reinterpret_cast<const float*>(&local_to_world_tf->value[0][0]));
		Entity::validatePtr(entity)->setTransform(tf);
		if (tapeRecord.has_value()) {
			tapeRecord->recordEntitySetPose(entity, local_to_world_tf);
		}
	});
}

RGL_API rgl_status_t
rgl_lidar_create(rgl_lidar_t* out_lidar,
		 const rgl_mat3x4f* ray_transforms,
		 int ray_transforms_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_create(out_lidar={}, ray_transforms={})",
			  (void*) out_lidar, repr(ray_transforms, ray_transforms_count));
		CHECK_ARG(out_lidar != nullptr);
		CHECK_ARG(ray_transforms != nullptr);
		CHECK_ARG(ray_transforms_count > 0);
		*out_lidar = Lidar::create(reinterpret_cast<const Mat3x4f*>(ray_transforms),
					   ray_transforms_count).get();
	});
}

RGL_API rgl_status_t
rgl_lidar_destroy(rgl_lidar_t lidar)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_destroy(lidar={})", (void*) lidar);
		CHECK_ARG(lidar != nullptr);
		Lidar::release(lidar);
	});
}

RGL_API rgl_status_t
rgl_lidar_set_range(rgl_lidar_t lidar, float range)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_set_range(lidar={}, range={})", (void*) lidar, range);
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(!std::isnan(range));
		CHECK_ARG(range > 0.0f);
		Lidar::validatePtr(lidar)->range = range;
	});
}

RGL_API rgl_status_t
rgl_lidar_set_pose(rgl_lidar_t lidar, const rgl_mat3x4f* local_to_world_tf)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_set_pose(lidar={}, local_to_world_tf={})", (void*) lidar, repr(local_to_world_tf));
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(local_to_world_tf != nullptr);
		Lidar::validatePtr(lidar)->lidarPose = Mat3x4f::fromRaw(&local_to_world_tf->value[0][0]);
	});
}

RGL_API rgl_status_t
rgl_lidar_raytrace_async(rgl_scene_t scene, rgl_lidar_t lidar)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_raytrace_async(scene={}, lidar={})", (void*) scene, (void*) lidar);
		CHECK_ARG(lidar != nullptr);
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		Lidar::validatePtr(lidar)->scheduleRaycast(Scene::validatePtr(scene));
	});
}

RGL_API rgl_status_t
rgl_lidar_get_output_size(rgl_lidar_t lidar, int* out_size)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_get_output_size(lidar={}, out_size={})", (void*) lidar, (void*) out_size);
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(out_size != nullptr);
		*out_size = Lidar::validatePtr(lidar)->getResultsSize();
	});
}

RGL_API rgl_status_t
rgl_lidar_get_output_data(rgl_lidar_t lidar, rgl_format_t format, void* out_data)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_get_output_data(lidar={}, format={}, out_data={})", (void*) lidar, (int) format,
			  out_data);
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
rgl_lidar_set_ring_indices(rgl_lidar_t lidar, const int* ring_ids, int ring_ids_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_set_ring_indices(lidar={}, ring_ids={})",
			  (void*) lidar, repr(ring_ids, ring_ids_count, 5));
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(ring_ids != nullptr);
		CHECK_ARG(ring_ids_count > 0);
		Lidar::validatePtr(lidar)->setRingIds(ring_ids, ring_ids_count);
	});
}

RGL_API rgl_status_t
rgl_lidar_set_gaussian_noise_params(rgl_lidar_t lidar,
				    rgl_angular_noise_type_t angular_noise_type,
				    float angular_noise_stddev,
				    float angular_noise_mean,
				    float distance_noise_stddev_base,
				    float distance_noise_stddev_rise_per_meter,
				    float distance_noise_mean)
{
	return rglSafeCall([&]() {
		RGL_DEBUG(
			"rgl_lidar_set_gaussian_noise_params(lidar={}, angular_noise_type={}, angular_noise_stddev={}, angular_noise_mean={}, distance_noise_stddev_base={}, distance_noise_stddev_rise_per_meter={}, distance_noise_mean={}",
			(void*) lidar, angular_noise_type, angular_noise_stddev, angular_noise_mean,
			distance_noise_stddev_base, distance_noise_stddev_rise_per_meter, distance_noise_mean);
		bool noiseTypeOK = angular_noise_type == RGL_ANGULAR_NOISE_TYPE_RAY_BASED ||
				   angular_noise_type == RGL_ANGULAR_NOISE_TYPE_HITPOINT_BASED;
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(noiseTypeOK);
		CHECK_ARG(angular_noise_stddev >= 0.0f);
		CHECK_ARG(distance_noise_stddev_base >= 0.0f);
		CHECK_ARG(distance_noise_stddev_rise_per_meter >= 0.0f);
		Lidar::validatePtr(lidar)->lidarNoiseParams = {
			.angularNoiseType = (rgl_angular_noise_type_t) angular_noise_type,
			.angularNoiseStDev = angular_noise_stddev,
			.angularNoiseMean = angular_noise_mean,
			.distanceNoiseStDevBase = distance_noise_stddev_base,
			.distanceNoiseStDevRisePerMeter = distance_noise_stddev_rise_per_meter,
			.distanceNoiseMean = distance_noise_mean
		};
	});
}

RGL_API rgl_status_t
rgl_lidar_set_post_raycast_transform(rgl_lidar_t lidar, const rgl_mat3x4f* transform)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_lidar_set_post_raycast_transform(lidar={}, transform={})", (void*) lidar,
			  repr(transform));
		CHECK_ARG(lidar != nullptr);
		CHECK_ARG(transform != nullptr);
		Lidar::validatePtr(lidar)->rosTransform = Mat3x4f::fromRaw(&transform->value[0][0]);
	});
}

RGL_API rgl_status_t
rgl_tape_record_begin(const char* path)
{
	return rglSafeCall([&]() {
		CHECK_ARG(path != nullptr);
		RGL_DEBUG("rgl_tape_record_begin(path={})", path);
		if (tapeRecord.has_value()) {
			throw RecordError("rgl_tape_record_begin: recording already active");
		} else {
			tapeRecord.emplace(path);
		}
	});
}

RGL_API rgl_status_t
rgl_tape_record_end()
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_tape_record_end()");
		if (!tapeRecord.has_value()) {
			throw RecordError("rgl_tape_record_end: no recording active");
		} else {
			tapeRecord.reset();
		}
	});
}

RGL_API rgl_status_t
rgl_tape_play(const char* path)
{
	return rglSafeCall([&]() {
		CHECK_ARG(path != nullptr);
		RGL_DEBUG("rgl_tape_play(path={})", path);
		if (tapeRecord.has_value()) {
			throw RecordError("rgl_tape_play: recording active");
		} else {
			TapePlay play(path);
		}
	});
}
}