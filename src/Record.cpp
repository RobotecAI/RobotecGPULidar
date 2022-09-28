#include "Record.h"

#define RGL_VERSION "rgl_version"
#define MESH_CREATE "rgl_mesh_create"
#define MESH_DESTROY "rgl_mesh_destroy"
#define MESH_UPDATE_VERTICES "rgl_mesh_update_vertices"
#define ENTITY_CREATE "rgl_entity_create"
#define ENTITY_DESTROY "rgl_entity_destroy"
#define ENTITY_SET_POSE "rgl_entity_set_pose"

#define YAML_EXTENSION ".yaml"
#define BIN_EXTENSION ".bin"

Record& Record::instance()
{
    static Record instance;
    return instance;
}

void Record::writeRGLVersion(YAML::Node& node) {
    YAML::Node rglVersion;
    rglVersion["major"] = RGL_VERSION_MAJOR;
    rglVersion["minor"] = RGL_VERSION_MINOR;
    rglVersion["patch"] = RGL_VERSION_PATCH;
    node[RGL_VERSION] = rglVersion;
}

void Record::start(const char* path)
{
    if (recordingNow) {
        throw RecordError("rgl_record_start: recording already active");
    }
    std::string pathYaml = std::string(path) + YAML_EXTENSION;
    std::string pathBin = std::string(path) + BIN_EXTENSION;
    recordingNow = true;
    currentOffset = 0;
    meshIdRecord.clear();
    entityIdRecord.clear();
    fileBin = fopen(pathBin.c_str(), "wb");
    if (nullptr == fileBin) {
        throw InvalidFilePath(fmt::format("rgl_record_start: could not open binary file: {}", pathBin));
    }
    fileYaml.open(pathYaml);
    yamlRoot = YAML::Node();
    Record::writeRGLVersion(yamlRoot);
    yamlRecording = yamlRoot["recording"];
}

void Record::stop()
{
    if (!recordingNow) {
        throw RecordError("rgl_record_stop: no recording active");
    }
    recordingNow = false;
    fclose(fileBin);
    fileYaml << yamlRoot;
    fileYaml.close();
}

void Record::yamlNodeAdd(YAML::Node& node, const char* name)
{
    YAML::Node nameNode;
    nameNode[name] = node;
    yamlRecording.push_back(nameNode);
}

template <class T> size_t Record::writeToBin(const T* source, size_t elemCount, FILE* file)
{
    size_t elemSize = sizeof(T);
    uint8_t remainder = (elemSize * elemCount) % 16;
    uint8_t bytesToAdd = (16 - remainder) % 16;

    fwrite(source, elemSize, elemCount, file);
    if (remainder != 0) {
        uint8_t zeros[16];
        fwrite(zeros, sizeof(uint8_t), bytesToAdd, file);
    }

    currentOffset += elemSize * elemCount;
    return currentOffset - elemSize * elemCount;
}

size_t Record::insertMeshRecord(rgl_mesh_t mesh)
{
    meshIdRecord.insert(std::make_pair(mesh, nextMeshId));
    nextMeshId++;
    return nextMeshId - 1;
}

size_t Record::insertEntityRecord(rgl_entity_t entity)
{
    entityIdRecord.insert(std::make_pair(entity, nextEntityId));
    nextEntityId++;
    return nextEntityId - 1;
}

void Record::recordMeshCreate(rgl_mesh_t* outMesh,
                              const rgl_vec3f* vertices,
                              int vertexCount,
                              const rgl_vec3i* indices,
                              int indexCount)
{
    if (!recordingNow) {
        throw RecordError("rgl_record_mesh_create: no recording active");
    }
    YAML::Node rglMeshCreate;
    rglMeshCreate["out_mesh"] = insertMeshRecord(*outMesh);
    rglMeshCreate["vertices"] = writeToBin<rgl_vec3f>(vertices, vertexCount, fileBin);
    rglMeshCreate["vertex_count"] = vertexCount;
    rglMeshCreate["indices"] = writeToBin<rgl_vec3i>(indices, indexCount, fileBin);
    rglMeshCreate["index_count"] = indexCount;
    yamlNodeAdd(rglMeshCreate, MESH_CREATE);
}

void Record::recordMeshDestroy(rgl_mesh_t mesh)
{
    if (!recordingNow) {
        throw RecordError("rgl_record_mesh_destroy: no recording active");
    }
    YAML::Node rglMeshDestroy;
    rglMeshDestroy["mesh"] = meshIdRecord[mesh];
    meshIdRecord.erase(mesh);
    yamlNodeAdd(rglMeshDestroy, MESH_DESTROY);
}

void Record::recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertexCount)
{
    if (!recordingNow) {
        throw RecordError("rgl_record_mesh_update_vertices: no recording active");
    }
    YAML::Node rglMeshUpdateVertices;
    rglMeshUpdateVertices["mesh"] = meshIdRecord[mesh];
    rglMeshUpdateVertices["vertices"] = writeToBin<rgl_vec3f>(vertices, vertexCount, fileBin);
    rglMeshUpdateVertices["vertex_count"] = vertexCount;
    yamlNodeAdd(rglMeshUpdateVertices, MESH_UPDATE_VERTICES);
}

// the scene parameter is not used since for now we can only use the default scene,
// left it here in case it will be useful later
void Record::recordEntityCreate(rgl_entity_t* outEntity, rgl_scene_t, rgl_mesh_t mesh)
{
    if (!recordingNow) {
        throw RecordError("rgl_record_entity_create: no recording active");
    }
    YAML::Node rglEntityCreate;
    rglEntityCreate["out_entity"] = insertEntityRecord(*outEntity);
    rglEntityCreate["scene"] = "default scene";
    rglEntityCreate["mesh"] = meshIdRecord[mesh];
    yamlNodeAdd(rglEntityCreate, ENTITY_CREATE);
}

void Record::recordEntityDestroy(rgl_entity_t entity)
{
    if (!recordingNow) {
        throw RecordError("rgl_record_entity_destroy: no recording active");
    }
    YAML::Node rglEntityDestroy;
    rglEntityDestroy["entity"] = entityIdRecord[entity];
    entityIdRecord.erase(entity);
    yamlNodeAdd(rglEntityDestroy, ENTITY_DESTROY);
}

void Record::recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* localToWorldTf)
{
    if (!recordingNow) {
        throw RecordError("rgl_record_entity_set_pose: no recording active");
    }
    YAML::Node rglEntitySetPose;
    rglEntitySetPose["entity"] = entityIdRecord[entity];
    rglEntitySetPose["local_to_world_tf"] = writeToBin<rgl_mat3x4f>(localToWorldTf, 1, fileBin);
    yamlNodeAdd(rglEntitySetPose, ENTITY_SET_POSE);
}

void Record::mmapInit(const char* path)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        throw InvalidFilePath(fmt::format("rgl_record_play: could not open binary file: {}", path));
    }

    struct stat statBuffer{};
    int err = fstat(fd, &statBuffer);
    if (err < 0) {
        throw RecordError("rgl_record_play: couldn't read bin file length");
    }

    fileMmap = (uint8_t*) mmap(nullptr, statBuffer.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    mmapSize = statBuffer.st_size;
    if (fileMmap == MAP_FAILED) {
        throw InvalidFilePath(fmt::format("rgl_record_play: could not open binary file: {}", path));
    }
    close(fd);
}

void Record::play(const char* path)
{
    if (recordingNow) {
        throw RecordError("rgl_record_play: recording active");
    }
    meshIdPlay.clear();
    entityIdPlay.clear();
    std::string pathYaml = std::string(path) + YAML_EXTENSION;
    std::string pathBin = std::string(path) + BIN_EXTENSION;
    mmapInit(pathBin.c_str());
    yamlRoot = YAML::LoadFile(pathYaml);
    if (yamlRoot[RGL_VERSION]["major"].as<int>() != RGL_VERSION_MAJOR ||
        yamlRoot[RGL_VERSION]["minor"].as<int>() != RGL_VERSION_MINOR) {
        throw RecordError("recording version does not match rgl version");
    }
    for (size_t i = 0; yamlRecording[i]; ++i) {
        auto functionCall = yamlRecording[i];
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

void Record::playMeshCreate(YAML::Node meshCreate)
{
    rgl_mesh_t mesh;
    rgl_mesh_create(&mesh,
                    reinterpret_cast<const rgl_vec3f*>(fileMmap + meshCreate["vertices"].as<size_t>()),
                    meshCreate["vertex_count"].as<int>(),
                    reinterpret_cast<const rgl_vec3i*>(fileMmap + meshCreate["indices"].as<size_t>()),
                    meshCreate["index_count"].as<int>());
    meshIdPlay.insert(std::make_pair(meshCreate["out_mesh"].as<size_t>(), mesh));
}

void Record::playMeshDestroy(YAML::Node meshDestroy)
{
    auto id = meshDestroy["mesh"].as<size_t>();
    rgl_mesh_destroy(meshIdPlay[id]);
    meshIdPlay.erase(id);
}

void Record::playMeshUpdateVertices(YAML::Node meshUpdateVertices)
{
    rgl_mesh_update_vertices(meshIdPlay[meshUpdateVertices["mesh"].as<size_t>()],
                             reinterpret_cast<const rgl_vec3f*>
                             (fileMmap + meshUpdateVertices["vertices"].as<size_t>()),
                             meshUpdateVertices["vertex_count"].as<int>());
}

void Record::playEntityCreate(YAML::Node entityCreate)
{
    rgl_entity_t entity;
    rgl_entity_create(&entity, nullptr, meshIdPlay[entityCreate["mesh"].as<size_t>()]);
    entityIdPlay.insert(std::make_pair(entityCreate["out_entity"].as<size_t>(), entity));
}

void Record::playEntityDestroy(YAML::Node entityDestroy)
{
    auto id = entityDestroy["entity"].as<size_t>();
    rgl_entity_destroy(entityIdPlay[id]);
    entityIdPlay.erase(id);
}

void Record::playEntitySetPose(YAML::Node entitySetPose)
{
    rgl_entity_set_pose(entityIdPlay[entitySetPose["entity"].as<size_t>()],
                        reinterpret_cast<const rgl_mat3x4f*>
                        (fileMmap + entitySetPose["local_to_world_tf"].as<size_t>()));
}