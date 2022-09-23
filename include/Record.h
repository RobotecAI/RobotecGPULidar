#pragma once
#include <rgl/api/experimental.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>
#include <cstdio>
#include <sys/mman.h>
#include <cstdlib>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

class Record {
private:
    bool recording_now = false;
    size_t current_offset = 0;
    size_t function_call_nr = 0;
    YAML::Node yaml_root;
    FILE* file_bin;
    uint8_t* file_mmap;
    size_t mmap_size;
    std::ofstream file_yaml;
    size_t next_mesh_id = 0;
    size_t next_entity_id = 0;
    std::map<rgl_mesh_t, size_t> mesh_id_record;
    std::map<rgl_entity_t, size_t> entity_id_record;
    std::map<size_t, rgl_mesh_t> mesh_id_play;
    std::map<size_t, rgl_entity_t> entity_id_play;

    void play_mesh_create(YAML::Node& mesh_create);
    void play_mesh_destroy(YAML::Node& mesh_destroy);
    void play_mesh_update_vertices(YAML::Node& mesh_update_vertices);

    void play_entity_create(YAML::Node& entity_create);
    void play_entity_destroy(YAML::Node& entity_destroy);
    void play_entity_set_pose(YAML::Node& entity_set_pose);

    void yaml_node_add(YAML::Node& node, const char* name);
    size_t write_to_bin(const void* source, size_t length, size_t number, FILE* file);
    size_t insert_mesh_record(rgl_mesh_t mesh);
    size_t insert_entity_record(rgl_entity_t entity);

    void open_mmap(const char* file_path);
public:
    bool recording() const;
    static Record& instance();

    void start(const char* file_path_yaml, const char* file_path_bin);
    void stop();
    void play(const char* file_path_yaml, const char* file_path_bin);

    void record_mesh_create(rgl_mesh_t *out_mesh,
                            const rgl_vec3f *vertices,
                            int vertex_count,
                            const rgl_vec3i *indices,
                            int index_count);
    void record_mesh_destroy(rgl_mesh_t mesh);
    void record_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f *vertices, int vertex_count);

    void record_entity_create(rgl_entity_t *out_entity, rgl_scene_t scene, rgl_mesh_t mesh);
    void record_entity_destroy(rgl_entity_t entity);
    void record_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f *local_to_world_tf);
};