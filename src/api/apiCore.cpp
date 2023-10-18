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

#include <rgl/api/core.h>
#include <rgl/api/extensions/tape.h>

#include <api/apiCommon.hpp>

#include <scene/Scene.hpp>
#include <scene/Entity.hpp>
#include <scene/Mesh.hpp>
#include <scene/Texture.hpp>

#include <graph/NodesCore.hpp>
#include <graph/GraphRunCtx.hpp>
#include <NvtxWrappers.hpp>

extern "C" {

RGL_API rgl_status_t
rgl_get_version_info(int32_t* out_major, int32_t* out_minor, int32_t* out_patch)
{
	// Short pre-changelog API version history:
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
	// 0.11.0: implement Graph API (Gaussian noise temporarily removed)
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_get_version_info(out_major={}, out_minor={}, out_patch={})",
		            (void*) out_major, (void*) out_minor, (void*) out_patch);
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

void TapePlayer::tape_get_version_info(const YAML::Node& yamlNode)
{
	int32_t out_major, out_minor, out_patch;
	rgl_get_version_info(&out_major, &out_minor, &out_patch);
	if (out_major != yamlNode[0].as<int32_t>()) {
		RGL_WARN("tape_get_version_info: out_major mismatch");
	}
	if (out_minor != yamlNode[1].as<int32_t>()) {
		RGL_WARN("tape_get_version_info: out_minor mismatch");
	}
	if (out_patch != yamlNode[2].as<int32_t>()) {
		RGL_WARN("tape_get_version_info: out_patch mismatch");
	}
}

RGL_API rgl_status_t
rgl_get_extension_info(rgl_extension_t extension, int32_t* out_available)
{
	auto status = rglSafeCall([&]() {
		CHECK_ARG(0 <= extension && extension < RGL_EXTENSION_COUNT);
		CHECK_ARG(out_available != nullptr);
		switch (extension) {
			case RGL_EXTENSION_PCL: *out_available = RGL_BUILD_PCL_EXTENSION; break;
			case RGL_EXTENSION_ROS2: *out_available = RGL_BUILD_ROS2_EXTENSION; break;
			case RGL_EXTENSION_UDP: *out_available = RGL_BUILD_UDP_EXTENSION; break;
			default: throw std::invalid_argument(fmt::format("queried unknown RGL extension: {}", extension));
		}
	});
	TAPE_HOOK(extension, out_available);
	return status;
}

void TapePlayer::tape_get_extension_info(const YAML::Node &yamlNode)
{
	int32_t out_available;
	int32_t recorded_available = yamlNode[1].as<int32_t>();
	rgl_get_extension_info(static_cast<rgl_extension_t>(yamlNode[0].as<int32_t>()), &out_available);
	if (out_available != recorded_available) {
		RGL_WARN(fmt::format("tape_get_extension_info: result mismatch, recorded={}, actual={}", recorded_available, out_available));
	}
}

RGL_API rgl_status_t
rgl_configure_logging(rgl_log_level_t log_level, const char* log_file_path, bool use_stdout)
{
	auto status = rglSafeCall([&]() {
		// No logging here, Logger::configure() will print logs after its initialization.
		CHECK_ARG(0 <= log_level && log_level <= 6);
		Logger::getOrCreate().configure(log_level, log_file_path, use_stdout);
	});
	TAPE_HOOK(log_level, log_file_path, use_stdout);
	return status;
}

void TapePlayer::tape_configure_logging(const YAML::Node& yamlNode)
{
	rgl_configure_logging(
		(rgl_log_level_t)yamlNode[0].as<int>(),
		yamlNode[1].as<std::string>().c_str(),
		yamlNode[2].as<bool>());
}

RGL_API void
rgl_get_last_error_string(const char** out_error_string)
{
	if (out_error_string == nullptr) {
		return;
	}
	// NO LOGGING HERE SINCE IT MAY THROW!
	*out_error_string = getLastErrorString();
}

RGL_API rgl_status_t
rgl_cleanup(void)
{
	auto status = rglSafeCall([&]() {
		// First, delete nodes, because there might be a thread accessing other structures.
		while (!Node::instances.empty()) {
			auto node = Node::instances.begin()->second;
			if (node->hasGraphRunCtx()) {
				node->getGraphRunCtx()->detachAndDestroy();
			}
			auto connectedNodes = node->disconnectConnectedNodes();
			for (auto&& nodeToRelease : connectedNodes) {
				Node::release(nodeToRelease.get());
			}
		}
		Entity::instances.clear();
		Mesh::instances.clear();
		Texture::instances.clear();
		Scene::defaultInstance()->clear();
	});
	TAPE_HOOK();
	return status;
}

void TapePlayer::tape_cleanup(const YAML::Node& yamlNode)
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
		RGL_API_LOG("rgl_mesh_create(out_mesh={}, vertices={}, indices={})",
		            (void*) out_mesh, repr(vertices, vertex_count), repr(indices, index_count, 1));
		CHECK_ARG(out_mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		CHECK_ARG(indices != nullptr);
		CHECK_ARG(index_count > 0);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		*out_mesh = Mesh::create(reinterpret_cast<const Vec3f*>(vertices),
		                         vertex_count,
		                         reinterpret_cast<const Vec3i*>(indices),
		                         index_count).get();
	});
	TAPE_HOOK(out_mesh, TAPE_ARRAY(vertices, vertex_count), vertex_count, TAPE_ARRAY(indices, index_count), index_count);
	return status;
}

void TapePlayer::tape_mesh_create(const YAML::Node& yamlNode)
{
	rgl_mesh_t mesh = nullptr;
	rgl_mesh_create(&mesh,
		reinterpret_cast<const rgl_vec3f*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>(),
		reinterpret_cast<const rgl_vec3i*>(fileMmap + yamlNode[3].as<size_t>()),
		yamlNode[4].as<int32_t>());
	tapeMeshes.insert(std::make_pair(yamlNode[0].as<TapeAPIObjectID>(), mesh));
}

RGL_API rgl_status_t
rgl_mesh_set_texture_coords(rgl_mesh_t mesh, const rgl_vec2f* uvs, int32_t uv_count) {
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_mesh_set_texture_coords(mesh={}, uvs={}, uv_count={})",
		            (void*) mesh, repr(uvs, uv_count), uv_count);
		CHECK_ARG(mesh != nullptr);
		CHECK_ARG(uvs != nullptr);
		CHECK_ARG(uv_count > 0);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		Mesh::validatePtr(mesh)->setTexCoords(reinterpret_cast<const Vec2f*>(uvs), uv_count);

	});
	TAPE_HOOK(mesh, TAPE_ARRAY(uvs, uv_count), uv_count);
	return status;
}

void TapePlayer::tape_mesh_set_texture_coords(const YAML::Node& yamlNode)
{
	rgl_mesh_set_texture_coords(tapeMeshes.at(yamlNode[0].as<TapeAPIObjectID>()),
		reinterpret_cast<const rgl_vec2f*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
}

RGL_API rgl_status_t
rgl_mesh_destroy(rgl_mesh_t mesh)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_mesh_destroy(mesh={})", (void*) mesh);
		CHECK_ARG(mesh != nullptr);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		Mesh::release(mesh);
	});
	TAPE_HOOK(mesh);
	return status;
}

void TapePlayer::tape_mesh_destroy(const YAML::Node& yamlNode)
{
	auto meshId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_mesh_destroy(tapeMeshes.at(meshId));
	tapeMeshes.erase(meshId);
}

RGL_API rgl_status_t
rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int32_t vertex_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_mesh_update_vertices(mesh={}, vertices={})", (void*) mesh, repr(vertices, vertex_count));
		CHECK_ARG(mesh != nullptr);
		CHECK_ARG(vertices != nullptr);
		CHECK_ARG(vertex_count > 0);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		Mesh::validatePtr(mesh)->updateVertices(reinterpret_cast<const Vec3f*>(vertices), vertex_count);
	});
	TAPE_HOOK(mesh, TAPE_ARRAY(vertices, vertex_count), vertex_count);
	return status;
}

void TapePlayer::tape_mesh_update_vertices(const YAML::Node& yamlNode)
{
	rgl_mesh_update_vertices(tapeMeshes.at(yamlNode[0].as<TapeAPIObjectID>()),
		reinterpret_cast<const rgl_vec3f*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
}

RGL_API rgl_status_t
rgl_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_entity_create(out_entity={}, scene={}, mesh={})", (void*) out_entity, (void*) scene, (void*) mesh);
		CHECK_ARG(out_entity != nullptr);
		CHECK_ARG(mesh != nullptr);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		*out_entity = Entity::create(Mesh::validatePtr(mesh)).get();
		Scene::validatePtr(scene)->addEntity(Entity::validatePtr(*out_entity));
	});
	TAPE_HOOK(out_entity, scene, mesh);
	return status;
}

void TapePlayer::tape_entity_create(const YAML::Node& yamlNode)
{
	rgl_entity_t entity = nullptr;
	rgl_entity_create(&entity,
		nullptr,  // TODO(msz-rai) support multiple scenes
		tapeMeshes.at(yamlNode[2].as<TapeAPIObjectID>()));
	tapeEntities.insert(std::make_pair(yamlNode[0].as<TapeAPIObjectID>(), entity));
}

RGL_API rgl_status_t
rgl_entity_destroy(rgl_entity_t entity)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_entity_destroy(entity={})", (void*) entity);
		CHECK_ARG(entity != nullptr);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		auto entitySafe = Entity::validatePtr(entity);
		entitySafe->scene->removeEntity(entitySafe);
		Entity::release(entity);
	});
	TAPE_HOOK(entity);
	return status;
}

void TapePlayer::tape_entity_destroy(const YAML::Node& yamlNode)
{
	auto entityId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_entity_destroy(tapeEntities.at(entityId));
	tapeEntities.erase(entityId);
}

RGL_API rgl_status_t
rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f* transform)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_entity_set_pose(entity={}, transform={})", (void*) entity, repr(transform, 1));
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(transform != nullptr);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		auto tf = Mat3x4f::fromRaw(reinterpret_cast<const float*>(&transform->value[0][0]));
		Entity::validatePtr(entity)->setTransform(tf);
	});
	TAPE_HOOK(entity, transform);
	return status;
}

void TapePlayer::tape_entity_set_pose(const YAML::Node& yamlNode)
{
	rgl_entity_set_pose(tapeEntities.at(yamlNode[0].as<TapeAPIObjectID>()),
		reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode[1].as<size_t>()));
}

RGL_API rgl_status_t
rgl_entity_set_id(rgl_entity_t entity, int32_t id)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_entity_set_id(entity={}, id={})", (void *) entity, id);
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(id != RGL_ENTITY_INVALID_ID);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		Entity::validatePtr(entity)->setId(id);
	});
	TAPE_HOOK(entity, id);
	return status;
}

void TapePlayer::tape_entity_set_id(const YAML::Node& yamlNode)
{
	rgl_entity_set_id(tapeEntities.at(yamlNode[0].as<TapeAPIObjectID>()), yamlNode[1].as<Field<ENTITY_ID_I32>::type>());
}

RGL_API rgl_status_t
rgl_entity_set_intensity_texture(rgl_entity_t entity, rgl_texture_t texture )
{
	auto status = rglSafeCall([&](){
		RGL_API_LOG("rgl_entity_set_intensity_texture(entity={}, texture={})", (void*) entity, (void*) texture);
		CHECK_ARG(entity != nullptr);
		CHECK_ARG(texture != nullptr);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		Entity::validatePtr(entity)->setIntensityTexture(Texture::validatePtr(texture));
	});

	TAPE_HOOK(entity, texture);
	return status;
}

void TapePlayer::tape_entity_set_intensity_texture(const YAML::Node &yamlNode)
{
	rgl_entity_set_intensity_texture(tapeEntities.at(yamlNode[0].as<TapeAPIObjectID>()),
		tapeTextures.at(yamlNode[1].as<TapeAPIObjectID>()));
}

RGL_API rgl_status_t
rgl_texture_create(rgl_texture_t* out_texture, const void* texels, int32_t width, int32_t height)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_texture_create(out_texture={}, width={}, height={})", (void*) out_texture, width, height);
		CHECK_ARG(out_texture != nullptr);
		CHECK_ARG(texels != nullptr);
		CHECK_ARG(width > 0);
		CHECK_ARG(height > 0);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		*out_texture = Texture::create(texels, width, height).get();
	});
	TAPE_HOOK(out_texture, TAPE_ARRAY(texels, (width * height * sizeof(TextureTexelFormat))), width, height);
	return status;
}

void TapePlayer::tape_texture_create(const YAML::Node& yamlNode)
{
	rgl_texture_t texture = nullptr;

	rgl_texture_create(&texture,
		reinterpret_cast<const void*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>(),
		yamlNode[3].as<int32_t>());

	tapeTextures.insert(std::make_pair(yamlNode[0].as<TapeAPIObjectID>(), texture));
}

RGL_API rgl_status_t
rgl_texture_destroy(rgl_texture_t texture)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_texture_destroy(texture={})", (void*) texture);
		CHECK_ARG(texture != nullptr);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		Texture::release(texture);
	});
	TAPE_HOOK(texture);
	return status;
}

void TapePlayer::tape_texture_destroy(const YAML::Node &yamlNode)
{
	auto textureId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_texture_destroy(tapeTextures.at(textureId));
	tapeTextures.erase(textureId);
}

RGL_API rgl_status_t
rgl_scene_set_time(rgl_scene_t scene, uint64_t nanoseconds)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_scene_set_time(scene={}, nanoseconds={})", (void*) scene, nanoseconds);
		GraphRunCtx::synchronizeAll(); // Prevent races with graph threads
		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}
		Scene::validatePtr(scene)->setTime(Time::nanoseconds(nanoseconds));
	});
	TAPE_HOOK(scene, nanoseconds);
	return status;
}

void TapePlayer::tape_scene_set_time(const YAML::Node& yamlNode)
{
	rgl_scene_set_time(nullptr,  // TODO(msz-rai) support multiple scenes
		yamlNode[1].as<uint64_t>());
}

RGL_API rgl_status_t
rgl_graph_run(rgl_node_t raw_node)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_run(node={})", repr(raw_node));
		CHECK_ARG(raw_node != nullptr);
		NvtxRange rg {NVTX_CAT_API, NVTX_COL_CALL, "rgl_graph_run"};
		auto node = Node::validatePtr(raw_node);
		if (!node->hasGraphRunCtx()) {
			GraphRunCtx::createAndAttach(node);
		}
		node->getGraphRunCtx()->executeAsync();
	});
	TAPE_HOOK(raw_node);
	return status;
}

void TapePlayer::tape_graph_run(const YAML::Node& yamlNode)
{
	rgl_graph_run(tapeNodes.at(yamlNode[0].as<TapeAPIObjectID>()));
}

RGL_API rgl_status_t
rgl_graph_destroy(rgl_node_t raw_node)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_destroy(node={})", repr(raw_node));
		CHECK_ARG(raw_node != nullptr);
		auto node = Node::validatePtr(raw_node);
		if (node->hasGraphRunCtx()) {
			node->getGraphRunCtx()->detachAndDestroy();
		}
		auto allNodes = node->disconnectConnectedNodes();
		for (auto&& nodeToRelease : allNodes) {
			Node::release(nodeToRelease.get());
		}
	});
	TAPE_HOOK(raw_node);
	return status;
}

void TapePlayer::tape_graph_destroy(const YAML::Node& yamlNode)
{
	rgl_node_t userNode = tapeNodes.at(yamlNode[0].as<TapeAPIObjectID>());
	std::set<Node::Ptr> graph = Node::validatePtr(userNode)->getConnectedNodes();
	std::set<TapeAPIObjectID> graphNodeIds;

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
		TapeAPIObjectID nodeId = *graphNodeIds.begin();
		tapeNodes.erase(nodeId);
		graphNodeIds.erase(nodeId);
	}
}

RGL_API rgl_status_t
rgl_graph_get_result_size(rgl_node_t node, rgl_field_t field, int32_t* out_count, int32_t* out_size_of)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_get_result_size(node={}, field={}, out_count={}, out_size_of={})", repr(node), field, (void*)out_count, (void*)out_size_of);
		CHECK_ARG(node != nullptr);
		NvtxRange rg {NVTX_CAT_API, NVTX_COL_CALL, "rgl_graph_get_result_size"};

		auto pointCloudNode = Node::validatePtr<IPointsNode>(node);
		pointCloudNode->waitForResults();
		auto elemCount = (int32_t) pointCloudNode->getPointCount();
		auto elemSize = (int32_t) pointCloudNode->getFieldPointSize(field);

		if (out_count != nullptr) { *out_count = elemCount; }
		if (out_size_of != nullptr) { *out_size_of = elemSize; }
	});
	TAPE_HOOK(node, field, out_count, out_size_of);
	return status;
}

void TapePlayer::tape_graph_get_result_size(const YAML::Node& yamlNode)
{
	int32_t out_count, out_size_of;
	rgl_graph_get_result_size(tapeNodes.at(yamlNode[0].as<TapeAPIObjectID>()),
		(rgl_field_t) yamlNode[1].as<int>(),
		&out_count,
		&out_size_of);

	auto tape_count = yamlNode[2].as<int32_t>();
	auto tape_size_of = yamlNode[3].as<int32_t>();
	// These warnings may be caused e.g. by gaussian noise (different seed -> different hits)
	if (out_count != tape_count) {
		RGL_WARN("tape_graph_get_result_size: actual count ({}) differs from tape count ({})", out_count, tape_count);
	}
	if (out_size_of != tape_size_of) {
		RGL_WARN("tape_graph_get_result_size: actual sizeof ({}) differs from tape sizeof ({})", out_size_of, tape_size_of);
	}
}

RGL_API rgl_status_t
rgl_graph_get_result_data(rgl_node_t node, rgl_field_t field, void* dst)
{
	static auto buffer = HostPinnedArray<char>::create();
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_get_result_data(node={}, field={}, data={})", repr(node), field, (void*) dst);
		CHECK_ARG(node != nullptr);
		CHECK_ARG(dst != nullptr);
		NvtxRange rg {NVTX_CAT_API, NVTX_COL_CALL, "rgl_graph_get_result_data"};

		auto pointCloudNode = Node::validatePtr<IPointsNode>(node);
		if (!pointCloudNode->hasField(field)) {
			auto msg = fmt::format("node {} does not provide field {}", pointCloudNode->getName(), toString(field));
			throw InvalidPipeline(msg);
		}

		// This part is a bit tricky:
		// The node is operating in a GraphRunCtx, i.e. in some CUDA stream.
		// All its DAAs are bound to that stream. The stream may be busy with processing other nodes.
		// We want to copy data without waiting until the aforementioned stream is synchronized.
		// Therefore, we want to invoke the copy from the source DAA in a separate stream - copy stream.
		// Take note, that since the source DAA is bound to a stream,
		// it could be unsafe to issue operations in a different stream.
		// E.g. If DAA was resized (data pointer already changed its value), but the reallocation didn't yet happen.
		// However, with the current architecture, we can reasonably expect that no DAA is modified by other Node than its owner.
		// Therefore, it should be sufficient to wait only for the stream operations issued by the given node (and, obviously, all previous).
		// After that, all pending operations on the source DAA are done, and it is safe to use it in the copy stream.
		pointCloudNode->waitForResults();

		// Temporary optimization to yield better results in AWSIM:
		// YieldNode (which has complementary part of this optimization) prefetches XYZ to host mem.
		// If we are asked for XYZ from YieldNode, we can use its host cache and immediately memcpy it.
		// TODO: This should work for any field in YieldNode (encountered test fails for other fields)
		if (auto yieldNode = std::dynamic_pointer_cast<YieldPointsNode>(pointCloudNode)) {
			if (field == XYZ_F32) {
				auto fieldArray = yieldNode->getXYZCache();
				size_t size = fieldArray->getCount() * fieldArray->getSizeOf();
				memcpy(dst, fieldArray->getRawReadPtr(), size);
				return;
			}
		}

		auto fieldArray = pointCloudNode->getFieldData(field);
		buffer->resize(fieldArray->getCount() * fieldArray->getSizeOf(), false, false);
		void* bufferDst = buffer->getWritePtr();
		const void* src = fieldArray->getRawReadPtr();
		size_t size = fieldArray->getCount() * fieldArray->getSizeOf();
		CHECK_CUDA(cudaMemcpyAsync(bufferDst, src, size, cudaMemcpyDefault, CudaStream::getCopyStream()->getHandle()));
		CHECK_CUDA(cudaStreamSynchronize(CudaStream::getCopyStream()->getHandle()));
		memcpy(dst, bufferDst, size);
	});
	TAPE_HOOK(node, field, dst);
	return status;
}

void TapePlayer::tape_graph_get_result_data(const YAML::Node& yamlNode)
{
	rgl_node_t node = tapeNodes.at(yamlNode[0].as<TapeAPIObjectID>());
	rgl_field_t field = (rgl_field_t) yamlNode[1].as<int>();
	int32_t out_count, out_size_of;
	rgl_graph_get_result_size(node, field, &out_count, &out_size_of);
	std::vector<char> tmpVec;
	tmpVec.reserve(out_count * out_size_of);
	rgl_graph_get_result_data(node, field, tmpVec.data());
}

RGL_API rgl_status_t
rgl_graph_node_add_child(rgl_node_t parent, rgl_node_t child)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_node_add_child(parent={}, child={})", repr(parent), repr(child));
		CHECK_ARG(parent != nullptr);
		CHECK_ARG(child != nullptr);
		CHECK_ARG(parent != child);

		Node::validatePtr(parent)->addChild(Node::validatePtr(child));
	});
	TAPE_HOOK(parent, child);
	return status;
}

void TapePlayer::tape_graph_node_add_child(const YAML::Node& yamlNode)
{
	rgl_graph_node_add_child(tapeNodes.at(yamlNode[0].as<TapeAPIObjectID>()), tapeNodes.at(yamlNode[1].as<TapeAPIObjectID>()));
}

RGL_API rgl_status_t
rgl_graph_node_remove_child(rgl_node_t parent, rgl_node_t child)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_node_remove_child(parent={}, child={})", repr(parent), repr(child));
		CHECK_ARG(parent != nullptr);
		CHECK_ARG(child != nullptr);

		Node::validatePtr(parent)->removeChild(Node::validatePtr(child));
	});
	TAPE_HOOK(parent, child);
	return status;
}

void TapePlayer::tape_graph_node_remove_child(const YAML::Node& yamlNode)
{
	rgl_graph_node_remove_child(tapeNodes.at(yamlNode[0].as<TapeAPIObjectID>()), tapeNodes.at(yamlNode[1].as<TapeAPIObjectID>()));
}

RGL_API rgl_status_t
rgl_graph_node_set_priority(rgl_node_t node, int32_t priority)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_node_set_priority(node={}, priority={})", repr(node), priority);
		CHECK_ARG(node != nullptr);

		Node::Ptr nodeShared = Node::validatePtr(node);
		if (nodeShared->hasGraphRunCtx()) {
			nodeShared->getGraphRunCtx()->synchronize();
		}
		nodeShared->setPriority(priority);
	});
	TAPE_HOOK(node, priority);
	return status;
}

void TapePlayer::tape_graph_node_set_priority(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.at(nodeId);
	auto priority = yamlNode[1].as<int32_t>();
	rgl_graph_node_set_priority(node, priority);
}

RGL_API rgl_status_t
rgl_graph_node_get_priority(rgl_node_t node, int32_t* out_priority)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_graph_node_get_priority(node={}, priority={})", repr(node), (void*) out_priority);
		CHECK_ARG(node != nullptr);
		CHECK_ARG(out_priority != nullptr);

		Node::Ptr nodeShared = Node::validatePtr(node);
		// No need to synchronize
		*out_priority = nodeShared->getPriority();
	});
	TAPE_HOOK(node, out_priority);
	return status;
}

void TapePlayer::tape_graph_node_get_priority(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.at(nodeId);
	auto tape_priority = yamlNode[1].as<int32_t>();
	int32_t out_priority;
	rgl_graph_node_set_priority(node, out_priority);
	if (tape_priority != out_priority) {
		RGL_WARN("tape_graph_node_get_priority: actual priority ({}) differs from tape priority ({})", out_priority, tape_priority);
	}
}

RGL_API rgl_status_t
rgl_node_rays_from_mat3x4f(rgl_node_t* node, const rgl_mat3x4f* rays, int32_t ray_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_rays_from_mat3x4f(node={}, rays={})", repr(node), repr(rays, ray_count));
		CHECK_ARG(node != nullptr);
		CHECK_ARG(rays != nullptr);
		CHECK_ARG(ray_count > 0);
		createOrUpdateNode<FromMat3x4fRaysNode>(node, reinterpret_cast<const Mat3x4f*>(rays), (size_t)ray_count);
	});
	TAPE_HOOK(node, TAPE_ARRAY(rays, ray_count), ray_count);
	return status;
}

void TapePlayer::tape_node_rays_from_mat3x4f(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_rays_from_mat3x4f(&node,
		reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_rays_set_ring_ids(rgl_node_t* node, const int32_t* ring_ids, int32_t ring_ids_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_rays_set_ring_ids(node={}, ring_ids={})", repr(node), repr(ring_ids, ring_ids_count));
                CHECK_ARG(node != nullptr);
                CHECK_ARG(ring_ids != nullptr);
		CHECK_ARG(ring_ids_count > 0);
		createOrUpdateNode<SetRingIdsRaysNode>(node, ring_ids, (size_t) ring_ids_count);
	});
	TAPE_HOOK(node, TAPE_ARRAY(ring_ids, ring_ids_count), ring_ids_count);
	return status;
}

void TapePlayer::tape_node_rays_set_ring_ids(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_rays_set_ring_ids(&node,
		reinterpret_cast<const int32_t*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_rays_set_range(rgl_node_t* node, const rgl_vec2f* ranges, int32_t ranges_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_rays_set_range(node={}, ranges={})", repr(node), repr(ranges, ranges_count));
		CHECK_ARG(node != nullptr);
		CHECK_ARG(ranges != nullptr);
		CHECK_ARG(ranges_count > 0);
		createOrUpdateNode<SetRangeRaysNode>(node, reinterpret_cast<const Vec2f*>(ranges), (size_t) ranges_count);
	});
	TAPE_HOOK(node, TAPE_ARRAY(ranges, ranges_count), ranges_count);
	return status;
}

void TapePlayer::tape_node_rays_set_range(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_rays_set_range(&node,
							reinterpret_cast<const rgl_vec2f*>(fileMmap + yamlNode[1].as<size_t>()),
							yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_rays_set_time_offsets(rgl_node_t* node, const float* offsets, int32_t offsets_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_rays_set_time_offsets(node={}, offsets={})", repr(node), repr(offsets, offsets_count));
				CHECK_ARG(node != nullptr);
				CHECK_ARG(offsets != nullptr);
				CHECK_ARG(offsets_count > 0);
		createOrUpdateNode<SetTimeOffsetsRaysNode>(node, offsets, (size_t) offsets_count);
	});
	TAPE_HOOK(node, TAPE_ARRAY(offsets, offsets_count), offsets_count);
	return status;
}

void TapePlayer::tape_node_rays_set_time_offsets(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_rays_set_time_offsets(&node,
		reinterpret_cast<const float*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_rays_transform(rgl_node_t* node, const rgl_mat3x4f* transform)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_rays_transform(node={}, transform={})", repr(node), repr(transform));
		CHECK_ARG(node != nullptr);
		CHECK_ARG(transform != nullptr);

		createOrUpdateNode<TransformRaysNode>(node, Mat3x4f::fromRGL(*transform));
	});
	TAPE_HOOK(node, transform);
	return status;
}

void TapePlayer::tape_node_rays_transform(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_rays_transform(&node, reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode[1].as<size_t>()));
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_transform(rgl_node_t* node, const rgl_mat3x4f* transform)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_transform(node={}, transform={})", repr(node), repr(transform));
                CHECK_ARG(node != nullptr);
                CHECK_ARG(transform != nullptr);

		createOrUpdateNode<TransformPointsNode>(node, Mat3x4f::fromRGL(*transform));
	});
	TAPE_HOOK(node, transform);
	return status;
}

void TapePlayer::tape_node_points_transform(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_transform(&node, reinterpret_cast<const rgl_mat3x4f*>(fileMmap + yamlNode[1].as<size_t>()));
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_raytrace(rgl_node_t* node, rgl_scene_t scene)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_raytrace(node={}, scene={})", repr(node), (void*) scene);
		CHECK_ARG(node != nullptr);

		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}

		createOrUpdateNode<RaytraceNode>(node, Scene::validatePtr(scene));
		// Clear velocity that could be set by rgl_node_raytrace_with_distortion
		Node::validatePtr<RaytraceNode>(*node)->setVelocity(nullptr, nullptr);
	});
	TAPE_HOOK(node, scene);
	return status;
}

void TapePlayer::tape_node_raytrace(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_raytrace(&node, nullptr);  // TODO(msz-rai) support multiple scenes
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_raytrace_with_distortion(rgl_node_t* node, rgl_scene_t scene, const rgl_vec3f* linear_velocity, const rgl_vec3f* angular_velocity)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_raytrace_with_distortion(node={}, scene={}, linear_velocity={}, angular_velocity={})",
		            repr(node), (void*) scene, repr(linear_velocity), repr(angular_velocity));
		CHECK_ARG(node != nullptr);
		CHECK_ARG(linear_velocity != nullptr);
		CHECK_ARG(angular_velocity != nullptr);

		if (scene == nullptr) {
			scene = Scene::defaultInstance().get();
		}

		createOrUpdateNode<RaytraceNode>(node, Scene::validatePtr(scene));
		Node::validatePtr<RaytraceNode>(*node)->setVelocity(reinterpret_cast<const Vec3f*>(linear_velocity),
		                                                    reinterpret_cast<const Vec3f*>(angular_velocity));
	});
	TAPE_HOOK(node, scene, linear_velocity, angular_velocity);
	return status;
}

void TapePlayer::tape_node_raytrace_with_distortion(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_raytrace_with_distortion(&node,
	                                  nullptr,  // TODO(msz-rai) support multiple scenes
	                                  reinterpret_cast<const rgl_vec3f*>(fileMmap + yamlNode[2].as<size_t>()),
	                                  reinterpret_cast<const rgl_vec3f*>(fileMmap + yamlNode[3].as<size_t>()));
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_format(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_format(node={}, fields={})", repr(node), repr(fields, field_count));
                CHECK_ARG(node != nullptr);
                CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<FormatPointsNode>(node, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlayer::tape_node_points_format(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_format(&node,
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_yield(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_pipeline_yield(node={}, fields={})", repr(node), repr(fields, field_count));
                CHECK_ARG(node != nullptr);
                CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<YieldPointsNode>(node, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlayer::tape_node_points_yield(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_yield(&node,
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_compact(rgl_node_t* node)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_compact(node={})", repr(node));
                CHECK_ARG(node != nullptr);

		createOrUpdateNode<CompactPointsNode>(node);
	});
	TAPE_HOOK(node);
	return status;
}

void TapePlayer::tape_node_points_compact(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_compact(&node);
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_spatial_merge(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_spatial_merge(node={}, fields={})", repr(node), repr(fields, field_count));
                CHECK_ARG(node != nullptr);
                CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<SpatialMergePointsNode>(node, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlayer::tape_node_points_spatial_merge(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_spatial_merge(&node,
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_temporal_merge(rgl_node_t* node, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_temporal_merge(node={}, fields={})", repr(node), repr(fields, field_count));
                CHECK_ARG(node != nullptr);
                CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<TemporalMergePointsNode>(node, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlayer::tape_node_points_temporal_merge(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes[nodeId] : nullptr;
	rgl_node_points_temporal_merge(&node,
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_points_from_array(rgl_node_t* node, const void* points, int32_t points_count, const rgl_field_t* fields, int32_t field_count)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_points_from_array(node={}, points=[{},{}], fields={})", repr(node), (void*)points, points_count, repr(fields, field_count));
		CHECK_ARG(node != nullptr);
		CHECK_ARG(points != nullptr);
		CHECK_ARG(points_count > 0);
		CHECK_ARG(fields != nullptr);
		CHECK_ARG(field_count > 0);

		createOrUpdateNode<FromArrayPointsNode>(node, points, points_count, std::vector<rgl_field_t>{fields, fields + field_count});
	});
	TAPE_HOOK(node, TAPE_ARRAY(static_cast<const char*>(points), points_count * getPointSize(std::vector<rgl_field_t>{fields, fields + field_count})),
	          points_count, TAPE_ARRAY(fields, field_count), field_count);
	return status;
}

void TapePlayer::tape_node_points_from_array(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_points_from_array(&node,
		reinterpret_cast<const void*>(fileMmap + yamlNode[1].as<size_t>()),
		yamlNode[2].as<int32_t>(),
		reinterpret_cast<const rgl_field_t*>(fileMmap + yamlNode[3].as<size_t>()),
		yamlNode[4].as<int32_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_gaussian_noise_angular_ray(rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_gaussian_noise_angular_ray(node={}, mean={}, stDev={}, rotation_axis={})", repr(node), mean, st_dev, rotation_axis);
		CHECK_ARG(node != nullptr);
		CHECK_ARG(st_dev >= 0);
		CHECK_ARG((rotation_axis == RGL_AXIS_X) || (rotation_axis == RGL_AXIS_Y) || (rotation_axis == RGL_AXIS_Z));

		createOrUpdateNode<GaussianNoiseAngularRaysNode>(node, mean, st_dev, rotation_axis);
	});
	TAPE_HOOK(node, mean, st_dev, rotation_axis);
	return status;
}

void TapePlayer::tape_node_gaussian_noise_angular_ray(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_gaussian_noise_angular_ray(&node,
		yamlNode[1].as<float>(),
		yamlNode[2].as<float>(),
		(rgl_axis_t)yamlNode[3].as<size_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_gaussian_noise_angular_hitpoint(rgl_node_t* node, float mean, float st_dev, rgl_axis_t rotation_axis)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_gaussian_noise_angular_hitpoint(node={}, mean={}, st_dev={}, rotation_axis={})", repr(node), mean, st_dev, rotation_axis);
                CHECK_ARG(node != nullptr);
                CHECK_ARG(st_dev >= 0);
		CHECK_ARG((rotation_axis == RGL_AXIS_X) || (rotation_axis == RGL_AXIS_Y) || (rotation_axis == RGL_AXIS_Z));

		createOrUpdateNode<GaussianNoiseAngularHitpointNode>(node, mean, st_dev, rotation_axis);
	});
	TAPE_HOOK(node, mean, st_dev, rotation_axis);
	return status;
}

void TapePlayer::tape_node_gaussian_noise_angular_hitpoint(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_gaussian_noise_angular_hitpoint(&node,
		yamlNode[1].as<float>(),
		yamlNode[2].as<float>(),
		(rgl_axis_t)yamlNode[3].as<size_t>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_node_gaussian_noise_distance(rgl_node_t* node, float mean, float st_dev_base, float st_dev_rise_per_meter)
{
	auto status = rglSafeCall([&]() {
		RGL_API_LOG("rgl_node_gaussian_noise_distance(node={}, mean={}, st_dev_base={}, st_dev_rise_per_meter={})", repr(node), mean, st_dev_base, st_dev_rise_per_meter);
                CHECK_ARG(node != nullptr);
                CHECK_ARG(st_dev_base >= 0);
		CHECK_ARG(st_dev_rise_per_meter >= 0);

		createOrUpdateNode<GaussianNoiseDistanceNode>(node, mean, st_dev_base, st_dev_rise_per_meter);
	});
	TAPE_HOOK(node, mean, st_dev_base, st_dev_rise_per_meter);
	return status;
}

void TapePlayer::tape_node_gaussian_noise_distance(const YAML::Node& yamlNode)
{
	auto nodeId = yamlNode[0].as<TapeAPIObjectID>();
	rgl_node_t node = tapeNodes.contains(nodeId) ? tapeNodes.at(nodeId) : nullptr;
	rgl_node_gaussian_noise_distance(&node,
		yamlNode[1].as<float>(),
		yamlNode[2].as<float>(),
		yamlNode[3].as<float>());
	tapeNodes.insert({nodeId, node});
}

RGL_API rgl_status_t
rgl_tape_record_begin(const char* path)
{
	/**
	 * Please be mindful that portions of this function have a copy in rglLazyInit()
	 * Therefore, any changes must be also applied there (or refactored into a separate function).
	 */
	#ifdef _WIN32
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_record_begin(path={})", path);
		throw RecordError("rgl_tape_record_begin() is not supported on Windows");
	});
	#else
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_record_begin(path={})", path);
		CHECK_ARG(path != nullptr);
		CHECK_ARG(path[0] != '\0');
		if (tapeRecorder.has_value()) {
			throw RecordError("rgl_tape_record_begin: recording already active");
		} else {
			tapeRecorder.emplace(path);
		}
	});
	#endif //_WIN32
}

RGL_API rgl_status_t
rgl_tape_record_end()
{
	#ifdef _WIN32
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_record_end()");
		throw RecordError("rgl_tape_record_end() is not supported on Windows");
	});
	#else
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_record_end()");
		if (!tapeRecorder.has_value()) {
			throw RecordError("rgl_tape_record_end: no recording active");
		} else {
			tapeRecorder.reset();
		}
	});
	#endif //_WIN32
}

RGL_API rgl_status_t
rgl_tape_record_is_active(bool* is_active)
{
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_record_is_active(is_active={}", (void*) is_active);
		CHECK_ARG(is_active != nullptr);
		*is_active = tapeRecorder.has_value();
	});
}

RGL_API rgl_status_t
rgl_tape_play(const char* path)
{
	#ifdef _WIN32
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_play(path={})", path);
		throw RecordError("rgl_tape_play() is not supported on Windows");
	});
	#else
	return rglSafeCall([&]() {
		RGL_API_LOG("rgl_tape_play(path={})", path);
		CHECK_ARG(path != nullptr);
		CHECK_ARG(path[0] != '\0');
		if (tapeRecorder.has_value()) {
			throw RecordError("rgl_tape_play: recording active");
		} else {
			TapePlayer player(path);
			player.playRealtime();
		}
	});
	#endif //_WIN32
}
}
