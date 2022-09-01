#include <cmath>
#include <spdlog/common.h>

#include <rgl/api/experimental.h>

#include <scene/Scene.hpp>
#include <scene/Entity.hpp>
#include <scene/Mesh.hpp>

#include <graph/Nodes.hpp>
#include <graph/graph.hpp>

#include <RGLExceptions.hpp>
#include <macros/visibility.h>

#include <repr.hpp>

#define CHECK_ARG(expr)                                                                          \
do if (!(expr)) {                                                                                \
    auto msg = fmt::format("RGL API Error: Invalid argument, condition unsatisfied: {}", #expr); \
    throw std::invalid_argument(msg);                                                            \
} while(0)

static rgl_status_t lastStatusCode = RGL_SUCCESS;
static std::optional<std::string> lastStatusString = std::nullopt;

static bool canContinueAfterStatus(rgl_status_t status)
{
	// Set constructor may throw, hence lazy initialization.
	static std::set recoverableErrors = {
		RGL_INVALID_ARGUMENT,
		RGL_INVALID_API_OBJECT,
		RGL_INVALID_PIPELINE,
		RGL_NOT_IMPLEMENTED
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
		}
		else {
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
	catch (spdlog::spdlog_ex &e) {
		return updateAPIState(RGL_LOGGING_ERROR, e.what());
	}
	catch (InvalidAPIObject &e) {
		return updateAPIState(RGL_INVALID_API_OBJECT, e.what());
	}
	catch (InvalidPipeline &e) {
		return updateAPIState(RGL_INVALID_PIPELINE, e.what());
	}
	catch (std::invalid_argument &e) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, e.what());
	}
	catch (std::exception &e) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, e.what());
	}
	catch (...) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, "exceptional exception");
	}
	return updateAPIState(RGL_SUCCESS);
}

template<typename NodeType, typename... Args>
void createOrUpdateNode(rgl_node_t* nodeRawPtr, rgl_node_t parentRaw, Args&&... args)
{
	CHECK_ARG(nodeRawPtr != nullptr);
	std::shared_ptr<NodeType> node;
	if (*nodeRawPtr == nullptr) {
		node = Node::create<NodeType>();
		if (parentRaw != nullptr) {
			node->addParent(Node::validatePtr(parentRaw));
		}
	}
	else {
		node = Node::validatePtr<NodeType>(*nodeRawPtr);
	}
	node->setParameters(args...);
	*nodeRawPtr = node.get();
}

extern "C" {

RGL_API rgl_status_t
rgl_get_version_info(int *out_major, int *out_minor, int *out_patch)
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
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_get_version_info(out_major={}, out_minor={}, out_patch={})", (void*) out_major, (void*) out_minor, (void*) out_patch);
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
rgl_get_last_error_string(const char **out_error_string)
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
		default:
			*out_error_string = "???";
			break;
	}
}

RGL_API rgl_status_t
rgl_cleanup(void)
{
	return rglSafeCall([&]() {
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
		Entity::instances.clear();
		Mesh::instances.clear();
		Scene::defaultInstance()->clear();
		while (!Node::instances.empty()) {
			// Note: destroyPipeline calls Node::release()
			Node::Ptr node = Node::instances.begin()->second;
			destroyGraph(node);
		}
	});
}

RGL_API rgl_status_t
rgl_mesh_create(rgl_mesh_t *out_mesh, const rgl_vec3f *vertices, int vertex_count, const rgl_vec3i *indices, int index_count)
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
	});
}

RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_mesh_destroy(mesh={})", (void*) mesh);
		CHECK_ARG(mesh != nullptr);
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
		Mesh::release(mesh);
	});
}

RGL_API rgl_status_t
rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f *vertices, int vertex_count)
{
	return rglSafeCall([&]() {
        RGL_DEBUG("rgl_mesh_update_vertices(mesh={}, vertices={})", (void*) mesh, repr(vertices, vertex_count));
		CHECK_ARG(mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		Mesh::validatePtr(mesh)->updateVertices(reinterpret_cast<const Vec3f*>(vertices), vertex_count);
	});
}

RGL_API rgl_status_t
rgl_entity_create(rgl_entity_t *out_entity, rgl_scene_t scene, rgl_mesh_t mesh)
{
	return rglSafeCall([&]() {
        RGL_DEBUG("rgl_entity_create(out_entity={}, scene={}, mesh={})", (void*) out_entity, (void*) scene, (void*) mesh);
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
        RGL_DEBUG("rgl_entity_destroy(entity={})", (void*) entity);
		CHECK_ARG(entity != nullptr);
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
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
rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f *local_to_world_tf)
{
	return rglSafeCall([&]() {
        RGL_DEBUG("rgl_entity_set_pose(entity={}, local_to_world_tf={})", (void*) entity, repr(local_to_world_tf, 1));
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(local_to_world_tf != nullptr);
		auto tf = Mat3x4f::fromRaw(reinterpret_cast<const float *>(&local_to_world_tf->value[0][0]));
		Entity::validatePtr(entity)->setTransform(tf);
	});
}

RGL_API rgl_status_t
rgl_graph_run(rgl_node_t node)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_run(node={})", repr(node));
		CHECK_ARG(node != nullptr);
		runGraph(Node::validatePtr(node));
	});
}

RGL_API rgl_status_t
rgl_graph_destroy(rgl_node_t node)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_destroy(node={})", repr(node));
		CHECK_ARG(node != nullptr);
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
		destroyGraph(Node::validatePtr(node));
	});
}

RGL_API rgl_status_t
rgl_node_use_rays_mat3x4f(rgl_node_t* node, rgl_node_t parent, const rgl_mat3x4f* rays, size_t ray_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_use_rays_mat3x4f(node={}, parent={}, rays={})", repr(node), repr(parent), repr(rays, ray_count));
		CHECK_ARG(rays != nullptr);
		CHECK_ARG(ray_count > 0);
		createOrUpdateNode<UseRaysMat3x4fNode>(node, parent, reinterpret_cast<const Mat3x4f*>(rays), ray_count);
	});
}

RGL_API rgl_status_t
rgl_node_transform_rays(rgl_node_t* nodeRawPtr, rgl_node_t parent, const rgl_mat3x4f* transform)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_transform_rays(node={}, parent={}, transform={})", repr(nodeRawPtr), repr(parent), repr(transform));
		CHECK_ARG(transform != nullptr);

		createOrUpdateNode<TransformRaysNode>(nodeRawPtr, parent, Mat3x4f::fromRGL(*transform));
	});
}


RGL_API rgl_status_t
rgl_node_transform_points(rgl_node_t* node, rgl_node_t parent, const rgl_mat3x4f* transform)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_transform_points(node={}, parent={}, transform={})", repr(node), repr(parent), repr(transform));
		CHECK_ARG(transform != nullptr);

		createOrUpdateNode<TransformPointsNode>(node, parent, Mat3x4f::fromRGL(*transform));
	});
}

RGL_API rgl_status_t
rgl_node_raytrace(rgl_node_t* node, rgl_node_t parent, rgl_scene_t scene, float range)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_raytrace(node={}, parent={}, scene={}, range={})", repr(node), repr(parent), (void*) scene, range);
		CHECK_ARG(!std::isnan(range));
		CHECK_ARG(range > 0.0f);

		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}

		createOrUpdateNode<RaytraceNode>(node, parent, Scene::validatePtr(scene), range);
	});
}

RGL_API rgl_status_t
rgl_node_format(rgl_node_t* node, rgl_node_t parent, rgl_field_t* outToken, const rgl_field_t* fields, int field_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_format(node={}, parent={}, fields={})", repr(node), repr(parent), repr(fields, field_count));
		CHECK_ARG(outToken != nullptr);
		CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		// TODO: future feature
		*outToken = RGL_FIELD_DYNAMIC_BASE,
		createOrUpdateNode<FormatNode>(node, parent, std::vector<rgl_field_t>{fields, fields + field_count});
	});
}

RGL_API rgl_status_t
rgl_node_yield_points(rgl_node_t* node, rgl_node_t parent, const rgl_field_t* fields, int field_count)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_pipeline_yield(node={}, parent={}, fields={})", repr(node), repr(parent), repr(fields, field_count));
		CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<YieldPointsNode>(node, parent, std::vector<rgl_field_t>{fields, fields + field_count});
	});
}

RGL_API rgl_status_t
rgl_node_compact(rgl_node_t* node, rgl_node_t parent)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_compact(node={}, parent={})", repr(node), repr(parent));

		createOrUpdateNode<CompactNode>(node, parent);
	});
}

RGL_API rgl_status_t
rgl_node_downsample(rgl_node_t* node, rgl_node_t parent, float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_downsample(node={}, parent={}, leaf=({}, {}, {}))", repr(node), repr(parent), leaf_size_x, leaf_size_y, leaf_size_z);

		createOrUpdateNode<DownSampleNode>(node, parent, Vec3f{leaf_size_x, leaf_size_y, leaf_size_z});
	});
}

RGL_API rgl_status_t
rgl_node_write_pcd_file(rgl_node_t* node, rgl_node_t parent, const char* file_path)
{
	return rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_write_pcd_file(node={}, parent={}, file={})", repr(node), repr(parent), file_path);
		CHECK_ARG(file_path != nullptr);
		CHECK_ARG(file_path[0] != '\0');

		createOrUpdateNode<WritePCDFileNode>(node, parent, file_path);
	});
}
}
