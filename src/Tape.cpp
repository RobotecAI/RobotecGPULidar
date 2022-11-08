#include "Tape.h"

std::optional<TapeRecord> tapeRecord;

TapeRecord::TapeRecord(const char* path)
{
	std::string pathYaml = std::string(path) + YAML_EXTENSION;
	std::string pathBin = std::string(path) + BIN_EXTENSION;
	fileBin = fopen(pathBin.c_str(), "wb");
	if (nullptr == fileBin) {
		throw InvalidFilePath(fmt::format("rgl_tape_record_begin: could not open binary file: {}", pathBin));
	}
	fileYaml.open(pathYaml);
	TapeRecord::recordRGLVersion(yamlRoot);
	yamlRecording = yamlRoot["recording"];
}

TapeRecord::~TapeRecord()
{
	fclose(fileBin);
	fileYaml << yamlRoot;
	fileYaml.close();
}

void TapeRecord::recordRGLVersion(YAML::Node& node)
{
	YAML::Node rglVersion;
	rglVersion["major"] = RGL_VERSION_MAJOR;
	rglVersion["minor"] = RGL_VERSION_MINOR;
	rglVersion["patch"] = RGL_VERSION_PATCH;
	node[RGL_VERSION] = rglVersion;
}

TapePlay::TapePlay(const char* path)
{
	std::string pathYaml = std::string(path) + YAML_EXTENSION;
	std::string pathBin = std::string(path) + BIN_EXTENSION;

	mmapInit(pathBin.c_str());
	yamlRoot = YAML::LoadFile(pathYaml);
	auto yamlRecording = yamlRoot["recording"];

	if (yamlRoot[RGL_VERSION]["major"].as<int>() != RGL_VERSION_MAJOR ||
	    yamlRoot[RGL_VERSION]["minor"].as<int>() != RGL_VERSION_MINOR) {
		throw RecordError("recording version does not match rgl version");
	}

	for (const auto& functionCall: yamlRecording) {
		if (functionCall[MESH_CREATE]) {
			playMeshCreate(functionCall[MESH_CREATE]);
		} else if (functionCall[MESH_DESTROY]) {
			playMeshDestroy(functionCall[MESH_DESTROY]);
		} else if (functionCall[MESH_UPDATE_VERTICES]) {
			playMeshUpdateVertices(functionCall[MESH_UPDATE_VERTICES]);
		} else if (functionCall[ENTITY_CREATE]) {
			playEntityCreate(functionCall[ENTITY_CREATE]);
		} else if (functionCall[ENTITY_DESTROY]) {
			playEntityDestroy(functionCall[ENTITY_DESTROY]);
		} else if (functionCall[ENTITY_SET_POSE]) {
			playEntitySetPose(functionCall[ENTITY_SET_POSE]);
		}
	}

	munmap(fileMmap, mmapSize);
}

void TapePlay::mmapInit(const char* path)
{
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not open binary file: {}", path));
	}

	struct stat staticBuffer{};
	int err = fstat(fd, &staticBuffer);
	if (err < 0) {
		throw RecordError("rgl_tape_play: couldn't read bin file length");
	}

	fileMmap = (uint8_t*) mmap(nullptr, staticBuffer.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	mmapSize = staticBuffer.st_size;
	if (fileMmap == MAP_FAILED) {
		throw InvalidFilePath(fmt::format("rgl_tape_play: could not open binary file: {}", path));
	}
	close(fd);
}

void TapePlay::playMeshCreate(YAML::Node meshCreate)
{
	rgl_mesh_t mesh;
	rgl_mesh_create(&mesh,
			reinterpret_cast<const rgl_vec3f*>(fileMmap + meshCreate["vertices_offset"].as<size_t>()),
			meshCreate["vertex_count"].as<int>(),
			reinterpret_cast<const rgl_vec3i*>(fileMmap + meshCreate["indices_offset"].as<size_t>()),
			meshCreate["index_count"].as<int>());
	meshId.insert(std::make_pair(meshCreate["mesh_id"].as<size_t>(), mesh));
}

void TapePlay::playMeshDestroy(YAML::Node meshDestroy)
{
	auto id = meshDestroy["mesh_id"].as<size_t>();
	rgl_mesh_destroy(meshId[id]);
	meshId.erase(id);
}

void TapePlay::playMeshUpdateVertices(YAML::Node meshUpdateVertices)
{
	rgl_mesh_update_vertices(meshId[meshUpdateVertices["mesh_id"].as<size_t>()],
				 reinterpret_cast<const rgl_vec3f*>
				 (fileMmap + meshUpdateVertices["vertices_offset"].as<size_t>()),
				 meshUpdateVertices["vertex_count"].as<int>());
}

void TapePlay::playEntityCreate(YAML::Node entityCreate)
{
	rgl_entity_t entity;
	rgl_entity_create(&entity, nullptr, meshId[entityCreate["mesh_id"].as<size_t>()]);
	entityId.insert(std::make_pair(entityCreate["entity_id"].as<size_t>(), entity));
}

void TapePlay::playEntityDestroy(YAML::Node entityDestroy)
{
	auto id = entityDestroy["entity_id"].as<size_t>();
	rgl_entity_destroy(entityId[id]);
	entityId.erase(id);
}

void TapePlay::playEntitySetPose(YAML::Node entitySetPose)
{
	rgl_entity_set_pose(entityId[entitySetPose["entity_id"].as<size_t>()],
			    reinterpret_cast<const rgl_mat3x4f*>
			    (fileMmap + entitySetPose["local_to_world_tf_offset"].as<size_t>()));
}