#pragma once

#include <fcntl.h>
#include <fstream>
#include <optional>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>

#include <spdlog/fmt/bundled/format.h>
#include <yaml-cpp/yaml.h>

#include <RGLExceptions.hpp>
#include <rgl/api/experimental.h>

#define BIN_EXTENSION ".bin"
#define YAML_EXTENSION ".yaml"

#define ENTITY_CREATE "rgl_entity_create"
#define ENTITY_DESTROY "rgl_entity_destroy"
#define ENTITY_SET_POSE "rgl_entity_set_pose"
#define MESH_CREATE "rgl_mesh_create"
#define MESH_DESTROY "rgl_mesh_destroy"
#define MESH_UPDATE_VERTICES "rgl_mesh_update_vertices"
#define RGL_VERSION "rgl_version"

class TapeRecord
{
	YAML::Node yamlRoot;
	YAML::Node yamlRecording;
	size_t currentOffset = 0;
	FILE* fileBin;
	std::ofstream fileYaml;
	size_t nextMeshId = 0;
	size_t nextEntityId = 0;
	std::unordered_map<rgl_mesh_t, size_t> meshId;
	std::unordered_map<rgl_entity_t, size_t> entityId;

	void yamlNodeAdd(YAML::Node& node, const char* name);

	template<typename T>
	size_t writeToBin(const T* source, size_t elemCount, FILE* file);

	size_t saveNewMeshId(rgl_mesh_t mesh);

	size_t saveNewEntityId(rgl_entity_t entity);

	static void recordRGLVersion(YAML::Node& node);

public:

	explicit TapeRecord(const char* path);

	~TapeRecord();

	void recordMeshCreate(rgl_mesh_t* outMesh,
			      const rgl_vec3f* vertices,
			      int vertexCount,
			      const rgl_vec3i* indices,
			      int indexCount);

	void recordMeshDestroy(rgl_mesh_t mesh);

	void recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertexCount);

	void recordEntityCreate(rgl_entity_t* outEntity, rgl_scene_t scene, rgl_mesh_t mesh);

	void recordEntityDestroy(rgl_entity_t entity);

	void recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* localToWorldTf);
};

class TapePlay
{
	YAML::Node yamlRoot;
	uint8_t* fileMmap{};
	size_t mmapSize{};
	std::unordered_map<size_t, rgl_mesh_t> meshId;
	std::unordered_map<size_t, rgl_entity_t> entityId;

	void mmapInit(const char* path);

	void playMeshCreate(YAML::Node meshCreate);

	void playMeshDestroy(YAML::Node meshDestroy);

	void playMeshUpdateVertices(YAML::Node meshUpdateVertices);

	void playEntityCreate(YAML::Node entityCreate);

	void playEntityDestroy(YAML::Node entityDestroy);

	void playEntitySetPose(YAML::Node entitySetPose);

public:

	explicit TapePlay(const char* path);
};

extern std::optional<TapeRecord> tapeRecord;