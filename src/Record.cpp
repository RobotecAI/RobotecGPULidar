#include "Record.h"

#define MESH_CREATE "rgl_mesh_create"
#define MESH_DESTROY "rgl_mesh_destroy"
#define MESH_UPDATE_VERTICES "rgl_mesh_update_vertices"
#define ENTITY_CREATE "rgl_entity_create"
#define ENTITY_DESTROY "rgl_entity_destroy"
#define ENTITY_SET_POSE "rgl_entity_set_pose"

Record& Record::instance() {
    static Record instance;
    return instance;
}

bool Record::recording() const {
    return recordingNow;
}

void Record::start(const char* file_path_yaml, const char* file_path_bin) {
    assert(!recordingNow);
    recordingNow = true;
    currentOffset = 0;
    meshIdRecord = std::map<rgl_mesh_t, size_t>();
    entityIdRecord = std::map<rgl_entity_t, size_t>();
    fileBin = fopen(file_path_bin, "wb");
    if (nullptr == fileBin) throw InvalidFilePath("couldn't open bin file");
    fileYaml.open(file_path_yaml);
    yamlRoot = YAML::Node();
    int major, minor, patch;
    rgl_get_version_info(&major, &minor, &patch);
    YAML::Node rgl_version;
    rgl_version["major"] = major;
    rgl_version["minor"] = minor;
    rgl_version["patch"] = patch;
    yamlRoot[0]["rgl_version"] = rgl_version;
    functionIndex = 1;
}

void Record::stop() {
    assert(recordingNow);
    recordingNow = false;
    fclose(fileBin);
    fileYaml << yamlRoot;
    fileYaml.close();
}

void Record::yamlNodeAdd(YAML::Node& node, const char* name) {
    yamlRoot[functionIndex][name] = node;
    functionIndex++;
}

size_t Record::writeToBin(const void* source, size_t length, size_t number, FILE* file) {
    // since all the elements to be written are 4 bytes long
    if ((length * number) % 16 != 0) number += ((length * number) % 16) / 4;
    fwrite(source, length, number, file);
    currentOffset += length * number;
    return currentOffset - length * number;
}

size_t Record::insertMeshRecord(rgl_mesh_t mesh) {
    meshIdRecord.insert(std::make_pair(mesh, nextMeshId));
    nextMeshId++;
    return nextMeshId - 1;
}

size_t Record::insertEntityRecord(rgl_entity_t entity) {
    entityIdRecord.insert(std::make_pair(entity, nextEntityId));
    nextEntityId++;
    return nextEntityId - 1;
}

void Record::recordMeshCreate(rgl_mesh_t* out_mesh,
                              const rgl_vec3f* vertices,
                              int vertex_count,
                              const rgl_vec3i* indices,
                              int index_count) {
    assert(recordingNow);
    YAML::Node rglMeshCreate;
    rglMeshCreate["out_mesh"] = insertMeshRecord(*out_mesh);
    rglMeshCreate["vertices"] = writeToBin(vertices, sizeof(rgl_vec3f), vertex_count, fileBin);
    rglMeshCreate["vertex_count"] = vertex_count;
    rglMeshCreate["indices"] = writeToBin(indices, sizeof(rgl_vec3i), index_count, fileBin);
    rglMeshCreate["index_count"] = index_count;
    yamlNodeAdd(rglMeshCreate, MESH_CREATE);
}

void Record::recordMeshDestroy(rgl_mesh_t mesh) {
    assert(recordingNow);
    YAML::Node rglMeshDestroy;
    rglMeshDestroy["mesh"] = meshIdRecord[mesh];
    yamlNodeAdd(rglMeshDestroy, MESH_DESTROY);
}

void Record::recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertex_count) {
    assert(recordingNow);
    YAML::Node rglMeshUpdateVertices;
    rglMeshUpdateVertices["mesh"] = meshIdRecord[mesh];
    rglMeshUpdateVertices["vertices"] = writeToBin(vertices, sizeof(rgl_vec3f), vertex_count, fileBin);
    rglMeshUpdateVertices["vertex_count"] = vertex_count;
    yamlNodeAdd(rglMeshUpdateVertices, MESH_UPDATE_VERTICES);
}

void Record::recordEntityCreate(rgl_entity_t* out_entity, rgl_mesh_t mesh) {
    assert(recordingNow);
    YAML::Node rglEntityCreate;
    rglEntityCreate["out_entity"] = insertEntityRecord(*out_entity);
    rglEntityCreate["scene"] = "default scene";
    rglEntityCreate["mesh"] = meshIdRecord[mesh];
    yamlNodeAdd(rglEntityCreate, ENTITY_CREATE);
}

void Record::recordEntityDestroy(rgl_entity_t entity) {
    assert(recordingNow);
    YAML::Node rglEntityDestroy;
    rglEntityDestroy["entity"] = entityIdRecord[entity];
    yamlNodeAdd(rglEntityDestroy, ENTITY_DESTROY);
}

void Record::recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf) {
    assert(recordingNow);
    YAML::Node rglEntitySetPose;
    rglEntitySetPose["entity"] = entityIdRecord[entity];
    rglEntitySetPose["local_to_world_tf"] = writeToBin(local_to_world_tf, sizeof(rgl_mat3x4f), 1, fileBin);
    yamlNodeAdd(rglEntitySetPose, ENTITY_SET_POSE);
}

void Record::mmapInit(const char* file_path) {
    int fd = open(file_path, O_RDONLY);
    if (fd < 0) throw InvalidFilePath("couldn't open bin file");

    struct stat statbuf{};
    int err = fstat(fd, &statbuf);
    if (err < 0) throw InvalidFilePath("couldn't read bin file stats");

    fileMmap = (uint8_t*) mmap(nullptr, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    mmapSize = statbuf.st_size;
    if (fileMmap == MAP_FAILED) throw InvalidFilePath("couldn't open bin file using mmap");
    close(fd);
}

void Record::play(const char* file_path_yaml, const char* file_path_bin) {
    assert(!recordingNow);
    meshIdPlay = std::map<size_t, rgl_mesh_t>();
    entityIdPlay = std::map<size_t, rgl_entity_t>();
    mmapInit(file_path_bin);
    yamlRoot = YAML::LoadFile(file_path_yaml);
    int major, minor, patch;
    rgl_get_version_info(&major, &minor, &patch);
    assert(yamlRoot[0]["rgl_version"]["major"].as<int>() == major &&
           yamlRoot[0]["rgl_version"]["minor"].as<int>() == minor);
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

void Record::playMeshCreate(YAML::Node mesh_create) {
    rgl_mesh_t mesh;
    rgl_mesh_create(&mesh,
                    reinterpret_cast<const rgl_vec3f*>(fileMmap + mesh_create["vertices"].as<size_t>()),
                    mesh_create["vertex_count"].as<int>(),
                    reinterpret_cast<const rgl_vec3i*>(fileMmap + mesh_create["indices"].as<size_t>()),
                    mesh_create["index_count"].as<int>());
    meshIdPlay.insert(std::make_pair(mesh_create["out_mesh"].as<size_t>(), mesh));
}

void Record::playMeshDestroy(YAML::Node mesh_destroy) {
    rgl_mesh_destroy(meshIdPlay[mesh_destroy["mesh"].as<size_t>()]);
}

void Record::playMeshUpdateVertices(YAML::Node mesh_update_vertices) {
    rgl_mesh_update_vertices(meshIdPlay[mesh_update_vertices["mesh"].as<size_t>()],
                             reinterpret_cast<const rgl_vec3f*>
                             (fileMmap + mesh_update_vertices["vertices"].as<size_t>()),
                             mesh_update_vertices["vertex_count"].as<int>());
}

void Record::playEntityCreate(YAML::Node entity_create) {
    rgl_entity_t entity;
    rgl_entity_create(&entity, nullptr, meshIdPlay[entity_create["mesh"].as<size_t>()]);
    entityIdPlay.insert(std::make_pair(entity_create["out_entity"].as<size_t>(), entity));
}

void Record::playEntityDestroy(YAML::Node entity_destroy) {
    rgl_entity_destroy(entityIdPlay[entity_destroy["entity"].as<size_t>()]);
}

void Record::playEntitySetPose(YAML::Node entity_set_pose) {
    rgl_entity_set_pose(entityIdPlay[entity_set_pose["entity"].as<size_t>()],
                        reinterpret_cast<const rgl_mat3x4f*>
                        (fileMmap + entity_set_pose["local_to_world_tf"].as<size_t>()));
}