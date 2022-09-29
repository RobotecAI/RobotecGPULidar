#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <unordered_map>
#include <optional>

#include <yaml-cpp/yaml.h>
#include <spdlog/fmt/bundled/format.h>

#include <RGLExceptions.hpp>
#include <rgl/api/experimental.h>

#define YAML_EXTENSION ".yaml"
#define BIN_EXTENSION ".bin"

#define RGL_VERSION "rgl_version"
#define MESH_CREATE "rgl_mesh_create"
#define MESH_DESTROY "rgl_mesh_destroy"
#define MESH_UPDATE_VERTICES "rgl_mesh_update_vertices"
#define ENTITY_CREATE "rgl_entity_create"
#define ENTITY_DESTROY "rgl_entity_destroy"
#define ENTITY_SET_POSE "rgl_entity_set_pose"

class RecordWriter {
    YAML::Node yamlRoot;
    YAML::Node yamlRecording = yamlRoot["recording"];
    size_t currentOffset = 0;
    FILE* fileBin;
    std::ofstream fileYaml;
    size_t nextMeshId = 0;
    size_t nextEntityId = 0;
    std::unordered_map<rgl_mesh_t, size_t> meshIdRecord;
    std::unordered_map<rgl_entity_t, size_t> entityIdRecord;

    void yamlNodeAdd(YAML::Node& node, const char* name);

    template <class T> size_t writeToBin(const T* source, size_t elemCount, FILE* file);

    size_t insertMeshRecord(rgl_mesh_t mesh);

    size_t insertEntityRecord(rgl_entity_t entity);

    static void writeRGLVersion(YAML::Node& node);

public:

    explicit RecordWriter(const char* path) {
        std::string pathYaml = std::string(path) + YAML_EXTENSION;
        std::string pathBin = std::string(path) + BIN_EXTENSION;
        fileBin = fopen(pathBin.c_str(), "wb");
        if (nullptr == fileBin) {
            throw InvalidFilePath(fmt::format("rgl_record_start: could not open binary file: {}", pathBin));
        }
        fileYaml.open(pathYaml);
        RecordWriter::writeRGLVersion(yamlRoot);
    }

    ~RecordWriter() {
        fclose(fileBin);
        fileYaml << yamlRoot;
        fileYaml.close();
    }

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

class RecordReader {
    YAML::Node yamlRoot;
    uint8_t* fileMmap{};
    size_t mmapSize{};
    std::unordered_map<size_t, rgl_mesh_t> meshIdPlay;
    std::unordered_map<size_t, rgl_entity_t> entityIdPlay;

    void mmapInit(const char* path);

    void playMeshCreate(YAML::Node meshCreate);

    void playMeshDestroy(YAML::Node meshDestroy);

    void playMeshUpdateVertices(YAML::Node meshUpdateVertices);

    void playEntityCreate(YAML::Node entityCreate);

    void playEntityDestroy(YAML::Node entityDestroy);

    void playEntitySetPose(YAML::Node entitySetPose);

public:

    explicit RecordReader(const char* path) {
        std::string pathYaml = std::string(path) + YAML_EXTENSION;
        std::string pathBin = std::string(path) + BIN_EXTENSION;

        mmapInit(pathBin.c_str());
        yamlRoot = YAML::LoadFile(pathYaml);
        auto yamlRecording = yamlRoot["recording"];

        if (yamlRoot[RGL_VERSION]["major"].as<int>() != RGL_VERSION_MAJOR ||
            yamlRoot[RGL_VERSION]["minor"].as<int>() != RGL_VERSION_MINOR) {
            throw RecordError("recording version does not match rgl version");
        }

        for (auto it = yamlRecording.begin(); it != yamlRecording.end(); it++) {
            if ((*it)[MESH_CREATE]) {
                playMeshCreate((*it)[MESH_CREATE]);
            } else if ((*it)[MESH_DESTROY]) {
                playMeshDestroy((*it)[MESH_DESTROY]);
            } else if ((*it)[MESH_UPDATE_VERTICES]) {
                playMeshUpdateVertices((*it)[MESH_UPDATE_VERTICES]);
            } else if ((*it)[ENTITY_CREATE]) {
                playEntityCreate((*it)[ENTITY_CREATE]);
            } else if ((*it)[ENTITY_DESTROY]) {
                playEntityDestroy((*it)[ENTITY_DESTROY]);
            } else if ((*it)[ENTITY_SET_POSE]) {
                playEntitySetPose((*it)[ENTITY_SET_POSE]);
            }
        }

        munmap(fileMmap, mmapSize);
    }
};

extern std::optional<RecordWriter> recordWriter;