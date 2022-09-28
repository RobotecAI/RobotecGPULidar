#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <unordered_map>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <RGLExceptions.hpp>
#include <rgl/api/experimental.h>

class Record {
    bool recordingNow = false;
    size_t currentOffset = 0;
    YAML::Node yamlRoot;
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

    void playMeshCreate(YAML::Node mesh_create);

    void playMeshDestroy(YAML::Node mesh_destroy);

    void playMeshUpdateVertices(YAML::Node mesh_update_vertices);

    void playEntityCreate(YAML::Node entity_create);

    void playEntityDestroy(YAML::Node entity_destroy);

    void playEntitySetPose(YAML::Node entity_set_pose);

    void yamlNodeAdd(YAML::Node& node, const char* name);

    template <class T> size_t writeToBin(const T* source, size_t elemCount, FILE* file);

    size_t insertMeshRecord(rgl_mesh_t mesh);

    size_t insertEntityRecord(rgl_entity_t entity);

    void mmapInit(const char* path);

    void writeRGLVersion(YAML::Node& node);

    const char* concat(const char* cString, std::string& string);
public:
    bool recording() const {return recordingNow; }

    static Record& instance();

    void start(const char* path);

    void stop();

    void play(const char* path);

    void recordMeshCreate(rgl_mesh_t* out_mesh,
                          const rgl_vec3f* vertices,
                          int vertex_count,
                          const rgl_vec3i* indices,
                          int index_count);

    void recordMeshDestroy(rgl_mesh_t mesh);

    void recordMeshUpdateVertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertex_count);

    void recordEntityCreate(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh);

    void recordEntityDestroy(rgl_entity_t entity);

    void recordEntitySetPose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf);
};