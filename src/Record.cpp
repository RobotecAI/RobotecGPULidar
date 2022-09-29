#include "Record.h"

std::optional<RecordWriter> recordWriter;

void RecordWriter::writeRGLVersion(YAML::Node& node) {
    YAML::Node rglVersion;
    rglVersion["major"] = RGL_VERSION_MAJOR;
    rglVersion["minor"] = RGL_VERSION_MINOR;
    rglVersion["patch"] = RGL_VERSION_PATCH;
    node[RGL_VERSION] = rglVersion;
}

void RecordWriter::yamlNodeAdd(YAML::Node& node, const char* name)
{
    YAML::Node nameNode;
    nameNode[name] = node;
    yamlRecording.push_back(nameNode);
}

template <class T> size_t RecordWriter::writeToBin(const T* source, size_t elemCount, FILE* file)
{
    size_t elemSize = sizeof(T);
    uint8_t remainder = (elemSize * elemCount) % 16;
    uint8_t bytesToAdd = (16 - remainder) % 16;

    fwrite(source, elemSize, elemCount, file);
    if (remainder != 0) {
        uint8_t zeros[16];
        fwrite(zeros, sizeof(uint8_t), bytesToAdd, file);
    }

    currentOffset += elemSize * elemCount + bytesToAdd;
    return currentOffset - elemSize * elemCount - bytesToAdd;
}

size_t RecordWriter::insertMeshRecord(rgl_mesh_t mesh)
{
    meshIdRecord.insert(std::make_pair(mesh, nextMeshId));
    nextMeshId++;
    return nextMeshId - 1;
}

size_t RecordWriter::insertEntityRecord(rgl_entity_t entity)
{
    entityIdRecord.insert(std::make_pair(entity, nextEntityId));
    nextEntityId++;
    return nextEntityId - 1;
}

void RecordWriter::recordMeshCreate(rgl_mesh_t* outMesh,
                                    const rgl_vec3f* vertices,
                                    int vertexCount,
                                    const rgl_vec3i* indices,
                                    int indexCount)
{
    YAML::Node rglMeshCreate;
    rglMeshCreate["out_mesh"] = insertMeshRecord(*outMesh);
    rglMeshCreate["vertices"] = writeToBin<rgl_vec3f>(vertices, vertexCount, fileBin);
    rglMeshCreate["vertex_count"] = vertexCount;
    rglMeshCreate["indices"] = writeToBin<rgl_vec3i>(indices, indexCount, fileBin);
    rglMeshCreate["index_count"] = indexCount;
    yamlNodeAdd(rglMeshCreate, MESH_CREATE);
}

void RecordWriter::recordMeshDestroy(rgl_mesh_t mesh)
{
    YAML::Node rglMeshDestroy;
    rglMeshDestroy["mesh"] = meshIdRecord[mesh];
    meshIdRecord.erase(mesh);
    yamlNodeAdd(rglMeshDestroy, MESH_DESTROY);
}

void RecordWriter::recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertexCount)
{
    YAML::Node rglMeshUpdateVertices;
    rglMeshUpdateVertices["mesh"] = meshIdRecord[mesh];
    rglMeshUpdateVertices["vertices"] = writeToBin<rgl_vec3f>(vertices, vertexCount, fileBin);
    rglMeshUpdateVertices["vertex_count"] = vertexCount;
    yamlNodeAdd(rglMeshUpdateVertices, MESH_UPDATE_VERTICES);
}

// the scene parameter is not used since for now we can only use the default scene,
// left it here in case it will be useful later
void RecordWriter::recordEntityCreate(rgl_entity_t* outEntity, rgl_scene_t, rgl_mesh_t mesh)
{
    YAML::Node rglEntityCreate;
    rglEntityCreate["out_entity"] = insertEntityRecord(*outEntity);
    rglEntityCreate["scene"] = "default scene";
    rglEntityCreate["mesh"] = meshIdRecord[mesh];
    yamlNodeAdd(rglEntityCreate, ENTITY_CREATE);
}

void RecordWriter::recordEntityDestroy(rgl_entity_t entity)
{
    YAML::Node rglEntityDestroy;
    rglEntityDestroy["entity"] = entityIdRecord[entity];
    entityIdRecord.erase(entity);
    yamlNodeAdd(rglEntityDestroy, ENTITY_DESTROY);
}

void RecordWriter::recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* localToWorldTf)
{
    YAML::Node rglEntitySetPose;
    rglEntitySetPose["entity"] = entityIdRecord[entity];
    rglEntitySetPose["local_to_world_tf"] = writeToBin<rgl_mat3x4f>(localToWorldTf, 1, fileBin);
    yamlNodeAdd(rglEntitySetPose, ENTITY_SET_POSE);
}

void RecordReader::mmapInit(const char* path)
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

void RecordReader::playMeshCreate(YAML::Node meshCreate)
{
    rgl_mesh_t mesh;
    rgl_mesh_create(&mesh,
                    reinterpret_cast<const rgl_vec3f*>(fileMmap + meshCreate["vertices"].as<size_t>()),
                    meshCreate["vertex_count"].as<int>(),
                    reinterpret_cast<const rgl_vec3i*>(fileMmap + meshCreate["indices"].as<size_t>()),
                    meshCreate["index_count"].as<int>());
    meshIdPlay.insert(std::make_pair(meshCreate["out_mesh"].as<size_t>(), mesh));
}

void RecordReader::playMeshDestroy(YAML::Node meshDestroy)
{
    auto id = meshDestroy["mesh"].as<size_t>();
    rgl_mesh_destroy(meshIdPlay[id]);
    meshIdPlay.erase(id);
}

void RecordReader::playMeshUpdateVertices(YAML::Node meshUpdateVertices)
{
    rgl_mesh_update_vertices(meshIdPlay[meshUpdateVertices["mesh"].as<size_t>()],
                             reinterpret_cast<const rgl_vec3f*>
                             (fileMmap + meshUpdateVertices["vertices"].as<size_t>()),
                             meshUpdateVertices["vertex_count"].as<int>());
}

void RecordReader::playEntityCreate(YAML::Node entityCreate)
{
    rgl_entity_t entity;
    rgl_entity_create(&entity, nullptr, meshIdPlay[entityCreate["mesh"].as<size_t>()]);
    entityIdPlay.insert(std::make_pair(entityCreate["out_entity"].as<size_t>(), entity));
}

void RecordReader::playEntityDestroy(YAML::Node entityDestroy)
{
    auto id = entityDestroy["entity"].as<size_t>();
    rgl_entity_destroy(entityIdPlay[id]);
    entityIdPlay.erase(id);
}

void RecordReader::playEntitySetPose(YAML::Node entitySetPose)
{
    rgl_entity_set_pose(entityIdPlay[entitySetPose["entity"].as<size_t>()],
                        reinterpret_cast<const rgl_mat3x4f*>
                        (fileMmap + entitySetPose["local_to_world_tf"].as<size_t>()));
}