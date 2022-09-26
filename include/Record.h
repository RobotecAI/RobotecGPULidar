#pragma once

#include <cassert>
#include <rgl/api/experimental.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <RGLExceptions.hpp>

class Record {
private:
    bool recordingNow = false;
    size_t currentOffset = 0;
    size_t functionIndex = 0;
    YAML::Node yamlRoot;
    FILE* fileBin;
    uint8_t* fileMmap;
    size_t mmapSize;
    std::ofstream fileYaml;
    size_t nextMeshId = 0;
    size_t nextEntityId = 0;
    std::map<rgl_mesh_t, size_t> meshIdRecord;
    std::map<rgl_entity_t, size_t> entityIdRecord;
    std::map<size_t, rgl_mesh_t> meshIdPlay;
    std::map<size_t, rgl_entity_t> entityIdPlay;

    void playMeshCreate(YAML::Node mesh_create);

    void playMeshDestroy(YAML::Node mesh_destroy);

    void playMeshUpdateVertices(YAML::Node mesh_update_vertices);

    void playEntityCreate(YAML::Node entity_create);

    void playEntityDestroy(YAML::Node entity_destroy);

    void playEntitySetPose(YAML::Node entity_set_pose);

    void yamlNodeAdd(YAML::Node& node, const char* name);

    size_t writeToBin(const void* source, size_t length, size_t number, FILE* file);

    size_t insertMeshRecord(rgl_mesh_t mesh);

    size_t insertEntityRecord(rgl_entity_t entity);

    void mmapInit(const char* file_path);

public:
    bool recording() const;

    static Record& instance();

    void start(const char* file_path_yaml, const char* file_path_bin);

    void stop();

    void play(const char* file_path_yaml, const char* file_path_bin);

    void recordMeshCreate(rgl_mesh_t* out_mesh,
                          const rgl_vec3f* vertices,
                          int vertex_count,
                          const rgl_vec3i* indices,
                          int index_count);

    void recordMeshDestroy(rgl_mesh_t mesh);

    void recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertex_count);

    void recordEntityCreate(rgl_entity_t* out_entity, rgl_mesh_t mesh);

    void recordEntityDestroy(rgl_entity_t entity);

    void recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf);
};