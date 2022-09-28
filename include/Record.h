#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <unordered_map>

#include <yaml-cpp/yaml.h>
#include <spdlog/fmt/bundled/format.h>

#include <RGLExceptions.hpp>
#include <rgl/api/experimental.h>

class Record {
    bool recordingNow = false;
    size_t currentOffset;
    YAML::Node yamlRoot;
    YAML::Node yamlRecording;
    FILE* fileBin;
    uint8_t* fileMmap;
    size_t mmapSize;
    std::ofstream fileYaml;
    size_t nextMeshId = 0;
    size_t nextEntityId = 0;
    std::unordered_map<rgl_mesh_t, size_t> meshIdRecord;
    std::unordered_map<rgl_entity_t, size_t> entityIdRecord;
    std::unordered_map<size_t, rgl_mesh_t> meshIdPlay;
    std::unordered_map<size_t, rgl_entity_t> entityIdPlay;

    void playMeshCreate(YAML::Node meshCreate);

    void playMeshDestroy(YAML::Node meshDestroy);

    void playMeshUpdateVertices(YAML::Node meshUpdateVertices);

    void playEntityCreate(YAML::Node entityCreate);

    void playEntityDestroy(YAML::Node entityDestroy);

    void playEntitySetPose(YAML::Node entitySetPose);

    void yamlNodeAdd(YAML::Node& node, const char* name);

    template <class T> size_t writeToBin(const T* source, size_t elemCount, FILE* file);

    size_t insertMeshRecord(rgl_mesh_t mesh);

    size_t insertEntityRecord(rgl_entity_t entity);

    void mmapInit(const char* path);

    static void writeRGLVersion(YAML::Node& node);

public:
    bool recording() const {return recordingNow; }

    static Record& instance();

    void start(const char* path);

    void stop();

    void play(const char* path);

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

namespace record {
    static bool recordingNow = false;

    class RecordWriter {

    };

    class RecordReader {

    };


}