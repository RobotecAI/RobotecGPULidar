// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <spdlog/common.h>

#include <rgl/api/core.h>
#include <rgl/api/extensions/tape.h>
#include <rgl/api/extensions/visualize.h>

#include <scene/Scene.hpp>
#include <scene/Entity.hpp>
#include <scene/Mesh.hpp>

#include <graph/Nodes.hpp>
#include <graph/graph.hpp>

#include <Tape.hpp>
#include <RGLExceptions.hpp>

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
	static auto& _ = Optix::instance(); // Trigger initialization on the first API call
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
	catch (InvalidPipeline &e) {
		return updateAPIState(RGL_INVALID_PIPELINE, e.what());
	}
	catch (InvalidFilePath& e) {
        return updateAPIState(RGL_INVALID_FILE_PATH, e.what());
    }
	catch (std::invalid_argument &e) {
		return updateAPIState(RGL_INVALID_ARGUMENT, e.what());
	}
	catch (RecordError& e) {
		return updateAPIState(RGL_TAPE_ERROR, e.what());
	}
	catch (std::exception& e) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, e.what());
	}
	catch (...) {
		return updateAPIState(RGL_INTERNAL_EXCEPTION, "exceptional exception");
	}
	return updateAPIState(RGL_SUCCESS);
}

template<typename NodeType, typename... Args>
void createOrUpdateNode(rgl_node_t* nodeRawPtr, Args&&... args)
{
	CHECK_ARG(nodeRawPtr != nullptr);
	std::shared_ptr<NodeType> node;
	if (*nodeRawPtr == nullptr) {
		node = Node::create<NodeType>();
	}
	else {
		node = Node::validatePtr<NodeType>(*nodeRawPtr);
	}
	node->setParameters(args...);
	*nodeRawPtr = node.get();
}

extern "C" {

RGL_API rgl_status_t
rgl_get_version_info(int32_t* out_major, int32_t* out_minor, int32_t* out_patch)
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
	// 0.10.0: tapeEntities can now share tapeMeshes
	// 0.10.1: API const correctness, added INVALID_OBJECT error, minor internal changes
	// 0.10.2: Fixed Lidar::getResults writing too many bytes
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_get_version_info(out_major={}, out_minor={}, out_patch={})", (void*) out_major,
			  (void*) out_minor, (void*) out_patch);
		CHECK_ARG(out_major != nullptr);
		CHECK_ARG(out_minor != nullptr);
		CHECK_ARG(out_patch != nullptr);
		*out_major = RGL_VERSION_MAJOR;
		*out_minor = RGL_VERSION_MINOR;
		*out_patch = RGL_VERSION_PATCH;
	});
	TAPE_HOOK(out_major, out_minor, out_patch);
	return status;
}

void TapePlay::tape_get_version_info(const YAML::Node& yamlNode)
{
	int32_t out_major, out_minor, out_patch;
	rgl_get_version_info(&out_major, &out_minor, &out_patch);
	if (out_major != yamlNode["0"].as<int32_t>()) RGL_WARN("tape_get_version_info: out_major mismatch");
	if (out_minor != yamlNode["1"].as<int32_t>()) RGL_WARN("tape_get_version_info: out_minor mismatch");
	if (out_patch != yamlNode["2"].as<int32_t>()) RGL_WARN("tape_get_version_info: out_patch mismatch");
}

RGL_API rgl_status_t
rgl_configure_logging(rgl_log_level_t log_level, const char* log_file_path, bool use_stdout)
{
	auto status = rglSafeCall([&]() {
		// No logging here, Logger::configure() will print logs after its initialization.
		CHECK_ARG(0 <= log_level && log_level <= 6);
		// Constructing string from nullptr is undefined behavior!
		auto logFilePath = log_file_path == nullptr ? std::nullopt : std::optional(log_file_path);
		Logger::instance().configure(log_level, logFilePath, use_stdout);
	});
	TAPE_HOOK(log_level, log_file_path, use_stdout);
	return status;
}

void TapePlay::tape_configure_logging(const YAML::Node& yamlNode)
{
	rgl_configure_logging((rgl_log_level_t)yamlNode["0"].as<int>(), yamlNode["1"].as<std::string>().c_str(), yamlNode["2"].as<bool>());
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
	auto status = rglSafeCall([&]() {
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
	TAPE_HOOK();
	return status;
}

void TapePlay::tape_cleanup(const YAML::Node& yamlNode)
{
	rgl_cleanup();
	tapeMeshes.clear();
	tapeEntities.clear();
	tapeNodes.clear();
}

RGL_API rgl_status_t
rgl_mesh_create(rgl_mesh_t* out_mesh, const rgl_vec3f* vertices, int32_t vertex_count, const rgl_vec3i* indices, int32_t index_count)
{
	auto status = rglSafeCall([&]() {
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
	TAPE_HOOK(out_mesh, TAPE_ARRAY(vertices, vertex_count), vertex_count, TAPE_ARRAY(indices, index_count), index_count);
	return status;
}

void TapePlay::tape_mesh_create(const YAML::Node& yamlNode)
{
	rgl_mesh_t mesh = nullptr;
	rgl_mesh_create(&mesh,
		reinterpret_cast<const rgl_vec3f*>(fileMmap + yamlNode["1"].as<size_t>()),
		yamlNode["2"].as<int>(),
		reinterpret_cast<const rgl_vec3i*>(fileMmap + yamlNode["3"].as<size_t>()),
		yamlNode["4"].as<int>());
	tapeMeshes.insert(std::make_pair(yamlNode["0"].as<size_t>(), mesh));
}

RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_mesh_destroy(mesh={})", (void*) mesh);
		CHECK_ARG(mesh != nullptr);
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
		Mesh::release(mesh);
	});
	TAPE_HOOK(mesh);
	return status;
}

void TapePlay::tape_mesh_destroy(const YAML::Node& yamlNode)
{
	auto meshId = yamlNode["0"].as<size_t>();
	rgl_mesh_destroy(tapeMeshes[meshId]);
	tapeMeshes.erase(meshId);
}

RGL_API rgl_status_t
rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int32_t vertex_count)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_mesh_update_vertices(mesh={}, vertices={})", (void*) mesh, repr(vertices, vertex_count));
		CHECK_ARG(mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		Mesh::validatePtr(mesh)->updateVertices(reinterpret_cast<const Vec3f*>(vertices), vertex_count);
	});
	TAPE_HOOK(mesh, TAPE_ARRAY(vertices, vertex_count), vertex_count);
	return status;
}

void TapePlay::tape_mesh_update_vertices(const YAML::Node& yamlNode)
{
	rgl_mesh_update_vertices(tapeMeshes[yamlNode["0"].as<size_t>()],
		reinterpret_cast<const rgl_vec3f*>(fileMmap + yamlNode["1"].as<size_t>()),
		yamlNode["2"].as<int>());
}

RGL_API rgl_status_t
rgl_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh)
{
	auto status = rglSafeCall([&]() {
        RGL_DEBUG("rgl_entity_create(out_entity={}, scene={}, mesh={})", (void*) out_entity, (void*) scene, (void*) mesh);
		CHECK_ARG(out_entity != nullptr);
		CHECK_ARG(mesh != nullptr);
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		*out_entity = Entity::create(Mesh::validatePtr(mesh)).get();
		Scene::validatePtr(scene)->addEntity(Entity::validatePtr(*out_entity));
	});
	TAPE_HOOK(out_entity, scene, mesh);
	return status;
}

void TapePlay::tape_entity_create(const YAML::Node& yamlNode)
{
	rgl_entity_t entity = nullptr;
	rgl_entity_create(&entity,
		nullptr,  // TODO(msz-rai) support multiple scenes
		tapeMeshes[yamlNode["2"].as<size_t>()]);
	tapeEntities.insert(std::make_pair(yamlNode["0"].as<size_t>(), entity));
}

RGL_API rgl_status_t
rgl_entity_destroy(rgl_entity_t entity)
{
	auto status = rglSafeCall([&]() {
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
	TAPE_HOOK(entity);
	return status;
}

void TapePlay::tape_entity_destroy(const YAML::Node& yamlNode)
{
	auto entityId = yamlNode["0"].as<size_t>();
	rgl_entity_destroy(tapeEntities[entityId]);
	tapeEntities.erase(entityId);
}

RGL_API rgl_status_t
rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_entity_set_pose(entity={}, local_to_world_tf={})", (void*) entity, repr(local_to_world_tf, 1));
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(local_to_world_tf != nullptr);
		auto tf = Mat3x4f::fromRaw(reinterpret_cast<const float*>(&local_to_world_tf->value[0][0]));
		Entity::validatePtr(entity)->setTransform(tf);
	});
	TAPE_HOOK(entity, local_to_world_tf);
	return status;
}

void TapePlay::tape_entity_set_pose(const YAML::Node& yamlNode)
{
	rgl_entity_set_pose(tapeEntities[yamlNode["0"].as<size_t>()],
		reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode["1"].as<size_t>()));
}

RGL_API rgl_status_t
rgl_graph_run(rgl_node_t node)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_run(node={})", repr(node));
		CHECK_ARG(node != nullptr);
		runGraph(Node::validatePtr(node));
	});
	TAPE_HOOK(node);
	return status;
}

void TapePlay::tape_graph_run(const YAML::Node& yamlNode)
{
	rgl_graph_run(tapeNodes[yamlNode["0"].as<size_t>()]);
}

RGL_API rgl_status_t
rgl_graph_destroy(rgl_node_t node)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_destroy(node={})", repr(node));
		CHECK_ARG(node != nullptr);
		CHECK_CUDA(cudaStreamSynchronize(nullptr));
		destroyGraph(Node::validatePtr(node));
	});
	TAPE_HOOK(node);
	return status;
}

void TapePlay::tape_graph_destroy(const YAML::Node& yamlNode)
{
	rgl_node_t userNode = tapeNodes[yamlNode["0"].as<size_t>()];
	std::set<Node::Ptr> graph = findConnectedNodes(Node::validatePtr(userNode));
	std::set<size_t> graphNodeIds;

	for (auto const& graphNode : graph) {
		for (auto const& [key, val] : tapeNodes) {
			auto tapeNodePtr = Node::validatePtr(val).get();
			if (graphNode.get() == tapeNodePtr) {
				graphNodeIds.insert(key);
				break;
			}
		}
	}

	rgl_graph_destroy(userNode);

	while (!graphNodeIds.empty()) {
		size_t nodeId = *graphNodeIds.begin();
		tapeNodes.erase(nodeId);
		graphNodeIds.erase(nodeId);
	}
}

RGL_API rgl_status_t
rgl_graph_get_result_size(rgl_node_t node, rgl_field_t field, int32_t* out_count, int32_t* out_size_of)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_get_result_size(node={}, field={}, out_count={}, out_size_of={})", repr(node), field, (void*)out_count, (void*)out_size_of);
		CHECK_ARG(node != nullptr);

		auto pointCloudNode = Node::validatePtr<IPointsNode>(node);
		int32_t elemCount = (int32_t)pointCloudNode->getPointCount();
		int32_t elemSize = (int32_t)pointCloudNode->getFieldPointSize(field);

		if (out_count != nullptr) { *out_count = elemCount; }
		if (out_size_of != nullptr) { *out_size_of = elemSize; }
	});
	TAPE_HOOK(node, field, out_count, out_size_of);
	return status;
}

void TapePlay::tape_graph_get_result_size(const YAML::Node& yamlNode)
{
	int32_t out_count, out_size_of;
	rgl_graph_get_result_size(tapeNodes[yamlNode["0"].as<size_t>()],
		(rgl_field_t)yamlNode["1"].as<int>(),
		&out_count,
		&out_size_of);

	if (out_count != yamlNode["2"].as<int32_t>()) RGL_WARN("tape_graph_get_result_size: out_count mismatch");
	if (out_size_of != yamlNode["3"].as<int32_t>()) RGL_WARN("tape_graph_get_result_size: out_size_of mismatch");
}

RGL_API rgl_status_t
rgl_graph_get_result_data(rgl_node_t node, rgl_field_t field, void* data)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_get_result_data(node={}, field={}, data={})", repr(node), field, (void*)data);
		CHECK_ARG(node != nullptr);
		CHECK_ARG(data != nullptr);

		auto pointCloudNode = Node::validatePtr<IPointsNode>(node);
		VArray::ConstPtr output = pointCloudNode->getFieldData(field, nullptr);

		// TODO: cudaMemcpyAsync + explicit sync can be used here (better behavior for multiple graphs)
		CHECK_CUDA(cudaMemcpy(data, output->getReadPtr(MemLoc::Device), output->getElemCount() * output->getElemSize(), cudaMemcpyDefault));
	});
	TAPE_HOOK(node, field, data);
	return status;
}

void TapePlay::tape_graph_get_result_data(const YAML::Node& yamlNode)
{
	rgl_node_t node = tapeNodes[yamlNode["0"].as<size_t>()];
	rgl_field_t field = (rgl_field_t)yamlNode["1"].as<int>();
	int32_t out_count, out_size_of;
	rgl_graph_get_result_size(node, field, &out_count, &out_size_of);
	std::vector<char> tmpVec;
	tmpVec.reserve(out_count * out_size_of);
	rgl_graph_get_result_data(node, field, tmpVec.data());
}

RGL_API rgl_status_t
rgl_graph_node_set_active(rgl_node_t node, bool active)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_node_set_active(node={}, active={})", repr(node), active);
		CHECK_ARG(node != nullptr);

		node->setActive(active);
	});
	TAPE_HOOK(node, active);
	return status;
}

void TapePlay::tape_graph_node_set_active(const YAML::Node& yamlNode)
{
	rgl_graph_node_set_active(tapeNodes[yamlNode["0"].as<size_t>()], yamlNode["1"].as<bool>());
}

RGL_API rgl_status_t
rgl_graph_node_add_child(rgl_node_t parent, rgl_node_t child)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_node_add_child(parent={}, child={})", repr(parent), repr(child));
		CHECK_ARG(parent != nullptr);
		CHECK_ARG(child != nullptr);

		parent->addChild(Node::validatePtr(child));
	});
	TAPE_HOOK(parent, child);
	return status;
}

void TapePlay::tape_graph_node_add_child(const YAML::Node& yamlNode)
{
	rgl_graph_node_add_child(tapeNodes[yamlNode["0"].as<size_t>()], tapeNodes[yamlNode["1"].as<size_t>()]);
}

RGL_API rgl_status_t
rgl_graph_node_remove_child(rgl_node_t parent, rgl_node_t child)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_graph_node_remove_child(parent={}, child={})", repr(parent), repr(child));
		CHECK_ARG(parent != nullptr);
		CHECK_ARG(child != nullptr);

		parent->removeChild(Node::validatePtr(child));
	});
	TAPE_HOOK(parent, child);
	return status;
}

void TapePlay::tape_graph_node_remove_child(const YAML::Node& yamlNode)
{
	rgl_graph_node_remove_child(tapeNodes[yamlNode["0"].as<size_t>()], tapeNodes[yamlNode["1"].as<size_t>()]);
}

RGL_API rgl_status_t
rgl_node_rays_from_mat3x4f(rgl_node_t* node, const rgl_mat3x4f* rays, int32_t ray_count)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_rays_from_mat3x4f(node={}, rays={})", repr(node), repr(rays, ray_count));
		CHECK_ARG(rays != nullptr);
		CHECK_ARG(ray_count > 0);
		createOrUpdateNode<FromMat3x4fRaysNode>(node, reinterpret_cast<const Mat3x4f*>(rays), (size_t)ray_count);
	});
	TAPE_HOOK(node, TAPE_ARRAY(rays, ray_count), ray_count);
	return status;
}

void TapePlay::tape_node_rays_from_mat3x4f(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_rays_from_mat3x4f(&node,
		reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode["1"].as<size_t>()),
		yamlNode["2"].as<int>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_rays_set_ring_ids(rgl_node_t* node, const int32_t* ring_ids, int32_t ring_ids_count)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_rays_set_ring_ids(node={}, ring_ids={})", repr(node), repr(ring_ids, ring_ids_count));
		CHECK_ARG(ring_ids != nullptr);
		CHECK_ARG(ring_ids_count > 0);
		createOrUpdateNode<SetRingIdsRaysNode>(node, ring_ids, (size_t)ring_ids_count);
	});
	TAPE_HOOK(node, TAPE_ARRAY(ring_ids, ring_ids_count), ring_ids_count);
	return status;
}

void TapePlay::tape_node_rays_set_ring_ids(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_rays_set_ring_ids(&node,
		reinterpret_cast<const int32_t*>(fileMmap + yamlNode["1"].as<size_t>()),
		yamlNode["2"].as<int>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_rays_transform(rgl_node_t* node, const rgl_mat3x4f* transform)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_rays_transform(node={}, transform={})", repr(node), repr(transform));
		CHECK_ARG(transform != nullptr);

		createOrUpdateNode<TransformRaysNode>(node, Mat3x4f::fromRGL(*transform));
	});
	TAPE_HOOK(node, transform);
	return status;
}

void TapePlay::tape_node_rays_transform(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_rays_transform(&node, reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode["1"].as<size_t>()));
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_transform(rgl_node_t* node, const rgl_mat3x4f* transform)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_transform(node={}, transform={})", repr(node), repr(transform));
		CHECK_ARG(transform != nullptr);

		createOrUpdateNode<TransformPointsNode>(node, Mat3x4f::fromRGL(*transform));
	});
	TAPE_HOOK(node, transform);
	return status;
}

void TapePlay::tape_node_points_transform(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_transform(&node, reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode["1"].as<size_t>()));
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_raytrace(rgl_node_t* node, rgl_scene_t scene, float range)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_raytrace(node={}, scene={}, range={})", repr(node), (void*) scene, range);
		CHECK_ARG(!std::isnan(range));
		CHECK_ARG(range > 0.0f);

		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}

		createOrUpdateNode<RaytraceNode>(node, Scene::validatePtr(scene), range);
	});
	TAPE_HOOK(node, scene, range);
	return status;
}

void TapePlay::tape_node_raytrace(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_raytrace(&node,
		nullptr,  // TODO(msz-rai) support multiple scenes
		yamlNode["2"].as<float>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_format(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_format(node={}, fields={})", repr(node), repr(fields, field_count));
		CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<FormatPointsNode>(node, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlay::tape_node_points_format(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_format(&node,
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode["1"].as<size_t>()),
		yamlNode["2"].as<int>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_yield(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_pipeline_yield(node={}, fields={})", repr(node), repr(fields, field_count));
		CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<YieldPointsNode>(node, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlay::tape_node_points_yield(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_yield(&node,
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode["1"].as<size_t>()),
		yamlNode["2"].as<int>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_compact(rgl_node_t* node)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_compact(node={})", repr(node));

		createOrUpdateNode<CompactPointsNode>(node);
	});
	TAPE_HOOK(node);
	return status;
}

void TapePlay::tape_node_points_compact(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_compact(&node);
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_downsample(rgl_node_t* node, float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_downsample(node={}, leaf=({}, {}, {}))", repr(node), leaf_size_x, leaf_size_y, leaf_size_z);

		createOrUpdateNode<DownSamplePointsNode>(node, Vec3f{leaf_size_x, leaf_size_y, leaf_size_z});
	});
	TAPE_HOOK(node, leaf_size_x, leaf_size_y, leaf_size_z);
	return status;
}

void TapePlay::tape_node_points_downsample(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_downsample(&node,
		yamlNode["1"].as<float>(),
		yamlNode["2"].as<float>(),
		yamlNode["3"].as<float>());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_write_pcd_file(rgl_node_t* node, const char* file_path)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_write_pcd_file(node={}, file={})", repr(node), file_path);
		CHECK_ARG(file_path != nullptr);
		CHECK_ARG(file_path[0] != '\0');

		createOrUpdateNode<WritePCDFilePointsNode>(node, file_path);
	});
	TAPE_HOOK(node, file_path);
	return status;
}

void TapePlay::tape_node_points_write_pcd_file(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_write_pcd_file(&node, yamlNode["1"].as<std::string>().c_str());
	tapeNodes.insert(std::make_pair(nodeId, node));
}

RGL_API rgl_status_t
rgl_node_points_visualize(rgl_node_t* node, const char* window_name, int32_t window_width, int32_t window_height, bool fullscreen)
{
	auto status = rglSafeCall([&]() {
		RGL_DEBUG("rgl_node_points_visualize(node={}, window_name={}, window_width={}, window_height={}, fullscreen={})",
		          repr(node), window_name, window_width, window_height, fullscreen);
		CHECK_ARG(window_name != nullptr);
		CHECK_ARG(window_name[0] != '\0');
		CHECK_ARG(window_width > 0);
		CHECK_ARG(window_height > 0);

		createOrUpdateNode<VisualizePointsNode>(node, window_name, window_width, window_height, fullscreen);
	});
	TAPE_HOOK(node, window_name, window_width, window_height, fullscreen);
	return status;
}

void TapePlay::tape_node_points_visualize(const YAML::Node& yamlNode)
{
	size_t nodeId = yamlNode["0"].as<size_t>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_visualize(&node,
		yamlNode["1"].as<std::string>().c_str(),
		yamlNode["2"].as<int>(),
		yamlNode["3"].as<int>(),
		yamlNode["4"].as<bool>());
	tapeNodes.insert(std::make_pair(nodeId, node));
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
