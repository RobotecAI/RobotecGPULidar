#include "Record.h"

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
    YAML::Node rgl_version;
    rgl_version["major"] = RGL_VERSION_MAJOR;
    rgl_version["minor"] = RGL_VERSION_MINOR;
    rgl_version["patch"] = RGL_VERSION_PATCH;
    YAML::Node name_node;
    name_node["rgl_version"] = rgl_version;
    node[0] = name_node;
}

void Record::start(const char* path)
{
    if (recordingNow) {
        throw RecordError("recording active");
    }
    std::string pathStringYaml(path);
    const char* path_yaml = (pathStringYaml.append(YAML_EXTENSION)).c_str();
    std::string pathStringBin(path);
    const char* path_bin = (pathStringBin.append(BIN_EXTENSION)).c_str();
//    std::cout << path_yaml << std::endl;
//    std::cout << path_bin << std::endl;
    recordingNow = true;
    currentOffset = 0;
    meshIdRecord.clear();
    entityIdRecord.clear();
    fileBin = fopen(path_bin, "wb");
    if (nullptr == fileBin) {
        throw InvalidFilePath(path_bin);
    }
    fileYaml.open(path_yaml);
    yamlRoot = YAML::Node();
    Record::writeRGLVersion(yamlRoot);
}

void Record::stop()
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    recordingNow = false;
    fclose(fileBin);
    fileYaml << yamlRoot;
    fileYaml.close();
}

void Record::yamlNodeAdd(YAML::Node& node, const char* name)
{
    YAML::Node name_node;
    name_node[name] = node;
    yamlRoot.push_back(name_node);
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

void Record::recordMeshCreate(rgl_mesh_t* out_mesh,
                              const rgl_vec3f* vertices,
                              int vertex_count,
                              const rgl_vec3i* indices,
                              int index_count)
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    YAML::Node rglMeshCreate;
    rglMeshCreate["out_mesh"] = insertMeshRecord(*out_mesh);
    rglMeshCreate["vertices"] = writeToBin<rgl_vec3f>(vertices, vertex_count, fileBin);
    rglMeshCreate["vertex_count"] = vertex_count;
    rglMeshCreate["indices"] = writeToBin<rgl_vec3i>(indices, index_count, fileBin);
    rglMeshCreate["index_count"] = index_count;
    yamlNodeAdd(rglMeshCreate, MESH_CREATE);
}

void Record::recordMeshDestroy(rgl_mesh_t mesh)
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    YAML::Node rglMeshDestroy;
    rglMeshDestroy["mesh"] = meshIdRecord[mesh];
    yamlNodeAdd(rglMeshDestroy, MESH_DESTROY);
}

void Record::recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertex_count)
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    YAML::Node rglMeshUpdateVertices;
    rglMeshUpdateVertices["mesh"] = meshIdRecord[mesh];
    rglMeshUpdateVertices["vertices"] = writeToBin<rgl_vec3f>(vertices, vertex_count, fileBin);
    rglMeshUpdateVertices["vertex_count"] = vertex_count;
    yamlNodeAdd(rglMeshUpdateVertices, MESH_UPDATE_VERTICES);
}

// the scene parameter is not used since for now we can only use the default scene,
// left it here in case it will be useful later
void Record::recordEntityCreate(rgl_entity_t* out_entity, rgl_scene_t, rgl_mesh_t mesh)
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    YAML::Node rglEntityCreate;
    rglEntityCreate["out_entity"] = insertEntityRecord(*out_entity);
    rglEntityCreate["scene"] = "default scene";
    rglEntityCreate["mesh"] = meshIdRecord[mesh];
    yamlNodeAdd(rglEntityCreate, ENTITY_CREATE);
}

void Record::recordEntityDestroy(rgl_entity_t entity)
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    YAML::Node rglEntityDestroy;
    rglEntityDestroy["entity"] = entityIdRecord[entity];
    yamlNodeAdd(rglEntityDestroy, ENTITY_DESTROY);
}

void Record::recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf)
{
    if (!recordingNow) {
        throw RecordError("no recording active");
    }
    YAML::Node rglEntitySetPose;
    rglEntitySetPose["entity"] = entityIdRecord[entity];
    rglEntitySetPose["local_to_world_tf"] = writeToBin<rgl_mat3x4f>(local_to_world_tf, 1, fileBin);
    yamlNodeAdd(rglEntitySetPose, ENTITY_SET_POSE);
}

void Record::mmapInit(const char* path)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        throw InvalidFilePath(path);
    }

    struct stat statbuf{};
    int err = fstat(fd, &statbuf);
    if (err < 0) {
        throw RecordError("couldn't read bin file length");
    }

    fileMmap = (uint8_t*) mmap(nullptr, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    mmapSize = statbuf.st_size;
    if (fileMmap == MAP_FAILED) {
        throw InvalidFilePath("couldn't open bin file using mmap");
    }
    close(fd);
}

void Record::play(const char* path)
{
    std::cout << "yaml ";
    if (recordingNow) {
        throw RecordError("recording active");
    }
    meshIdPlay.clear();
    entityIdPlay.clear();
    std::string pathStringYaml(path);
    const char* path_yaml = (pathStringYaml.append(YAML_EXTENSION)).c_str();
    std::string pathStringBin(path);
    const char* path_bin = (pathStringBin.append(BIN_EXTENSION)).c_str();
    std::cout << "yaml " << path_yaml << std::endl;
    std::cout << "yaml " << path_bin << std::endl;
    mmapInit(path_bin);
    yamlRoot = YAML::LoadFile(path_yaml);
    if (yamlRoot[0]["rgl_version"]["major"].as<int>() != RGL_VERSION_MAJOR ||
        yamlRoot[0]["rgl_version"]["minor"].as<int>() != RGL_VERSION_MINOR) {
        throw RecordError("recording version does not match rgl version");
    }
    for (size_t i = 1; yamlRoot[i]; ++i) {
        auto functionCall = yamlRoot[i];
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

void Record::playMeshCreate(YAML::Node mesh_create)
{
    rgl_mesh_t mesh;
    rgl_mesh_create(&mesh,
                    reinterpret_cast<const rgl_vec3f*>(fileMmap + mesh_create["vertices"].as<size_t>()),
                    mesh_create["vertex_count"].as<int>(),
                    reinterpret_cast<const rgl_vec3i*>(fileMmap + mesh_create["indices"].as<size_t>()),
                    mesh_create["index_count"].as<int>());
    meshIdPlay.insert(std::make_pair(mesh_create["out_mesh"].as<size_t>(), mesh));
}

void Record::playMeshDestroy(YAML::Node mesh_destroy)
{
    rgl_mesh_destroy(meshIdPlay[mesh_destroy["mesh"].as<size_t>()]);
}

void Record::playMeshUpdateVertices(YAML::Node mesh_update_vertices)
{
    rgl_mesh_update_vertices(meshIdPlay[mesh_update_vertices["mesh"].as<size_t>()],
                             reinterpret_cast<const rgl_vec3f*>
                             (fileMmap + mesh_update_vertices["vertices"].as<size_t>()),
                             mesh_update_vertices["vertex_count"].as<int>());
}

void Record::playEntityCreate(YAML::Node entity_create)
{
    rgl_entity_t entity;
    rgl_entity_create(&entity, nullptr, meshIdPlay[entity_create["mesh"].as<size_t>()]);
    entityIdPlay.insert(std::make_pair(entity_create["out_entity"].as<size_t>(), entity));
}

void Record::playEntityDestroy(YAML::Node entity_destroy)
{
    rgl_entity_destroy(entityIdPlay[entity_destroy["entity"].as<size_t>()]);
}

void Record::playEntitySetPose(YAML::Node entity_set_pose)
{
    rgl_entity_set_pose(entityIdPlay[entity_set_pose["entity"].as<size_t>()],
                        reinterpret_cast<const rgl_mat3x4f*>
                        (fileMmap + entity_set_pose["local_to_world_tf"].as<size_t>()));
}