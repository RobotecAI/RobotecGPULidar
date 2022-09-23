#include <Record.h>
#include <cassert>

#include <iostream>

#define FUNCTION_CALL "function_call_"
#define MESH_CREATE "rgl_mesh_create"
#define MESH_DESTROY "rgl_mesh_destroy"
#define MESH_UPDATE_VERTICES "rgl_mesh_update_vertices"
#define ENTITY_CREATE "rgl_entity_create"
#define ENTITY_DESTROY "rgl_entity_destroy"
#define ENTITY_SET_POSE "rgl_entity_set_pose"

Record &Record::instance()
{
    static Record instance;
    return instance;
}

bool Record::recording() const {
    return recording_now;
}

void Record::start(const char* file_path_yaml, const char* file_path_bin) {
    assert(!recording_now);
    recording_now = true;
    current_offset = 0;
    function_call_nr = 0;
    mesh_id_record = std::map<rgl_mesh_t, size_t>();
    entity_id_record = std::map<rgl_entity_t, size_t>();
    file_bin = fopen(file_path_bin, "wb");
    if (nullptr == file_bin) exit (1);
    file_yaml.open(file_path_yaml);
    yaml_root = YAML::Node();
}

void Record::stop() {
    assert(recording_now);
    recording_now = false;
    fclose(file_bin);
    file_yaml << yaml_root;
    std::cout << yaml_root;
    file_yaml << "DEBUG";
    file_yaml.close();
}

void Record::yaml_node_add(YAML::Node& node) {
    yaml_root[FUNCTION_CALL + std::to_string(function_call_nr)] = node;
    function_call_nr++;
}

size_t Record::write_to_bin(const void* source, size_t length, size_t number, FILE* file) {
    fwrite(source, length, number, file);
    current_offset += length * number;
    return current_offset - length * number;
}

size_t Record::insert_mesh_record(rgl_mesh_t mesh) {
    mesh_id_record.insert(std::make_pair(mesh, next_mesh_id));
    next_mesh_id++;
    return next_mesh_id - 1;
}

size_t Record::insert_entity_record(rgl_entity_t entity) {
    entity_id_record.insert(std::make_pair(entity, next_entity_id));
    next_entity_id++;
    return next_entity_id - 1;
}

void Record::record_mesh_create(rgl_mesh_t* out_mesh,
                           const rgl_vec3f* vertices,
                           int vertex_count,
                           const rgl_vec3i* indices,
                           int index_count) {
    YAML::Node rgl_mesh_create;
    rgl_mesh_create["out_mesh"] = insert_mesh_record(*out_mesh);
    rgl_mesh_create["vertices"] = write_to_bin(vertices, sizeof(rgl_vec3f), vertex_count, file_bin);
    rgl_mesh_create["vertex_count"] = vertex_count;
    rgl_mesh_create["indices"] = write_to_bin(indices, sizeof(rgl_vec3i), index_count, file_bin);
    rgl_mesh_create["index_count"] = index_count;
    yaml_node_add(rgl_mesh_create);
}

void Record::record_mesh_destroy(rgl_mesh_t mesh) {
    YAML::Node rgl_mesh_destroy;
    rgl_mesh_destroy["mesh"] = mesh_id_record[mesh];
    yaml_node_add(rgl_mesh_destroy);
}

void Record::record_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f* vertices, int vertex_count) {
    YAML::Node rgl_mesh_update_vertices;
    rgl_mesh_update_vertices["mesh"] = mesh_id_record[mesh];
    rgl_mesh_update_vertices["vertices"] = write_to_bin(vertices, sizeof(rgl_vec3f), vertex_count, file_bin);
    rgl_mesh_update_vertices["vertex_count"] = vertex_count;
    yaml_node_add(rgl_mesh_update_vertices);
}

void Record::record_entity_create(rgl_entity_t* out_entity, rgl_scene_t scene, rgl_mesh_t mesh) {
    YAML::Node rgl_entity_create;
    rgl_entity_create["out_entity"] = insert_entity_record(*out_entity);
    rgl_entity_create["scene"] = "default scene";
    rgl_entity_create["mesh"] = mesh_id_record[mesh];
    yaml_node_add(rgl_entity_create);
}

void Record::record_entity_destroy(rgl_entity_t entity) {
    YAML::Node rgl_entity_destroy;
    rgl_entity_destroy["entity"] = entity_id_record[entity];
    yaml_node_add(rgl_entity_destroy);
}

void Record::record_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f* local_to_world_tf) {
    YAML::Node rgl_entity_set_pose;
    rgl_entity_set_pose["entity"] = entity_id_record[entity];
    rgl_entity_set_pose["local_to_world_tf"] = write_to_bin(local_to_world_tf, sizeof(rgl_mat3x4f), 1, file_bin);
    yaml_node_add(rgl_entity_set_pose);
}

void Record::open_mmap(const char* file_path) {
    int fd = open(file_path, O_RDONLY);
    if(fd < 0){
        printf("\n\"%s \" could not open\n",
               file_path);
        exit(1);
    }

    struct stat statbuf{};
    int err = fstat(fd, &statbuf);
    if(err < 0){
        printf("\n\"%s \" could not open\n",
               file_path);
        exit(2);
    }

    file_mmap = (uint8_t*)mmap(nullptr, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    mmap_size = statbuf.st_size;
    if(file_mmap == MAP_FAILED){
        printf("Mapping Failed\n");
        exit(1);
    }
    close(fd);
}

void Record::play(const char* file_path_yaml, const char* file_path_bin) {
    assert(!recording_now);
    mesh_id_play = std::map<size_t, rgl_mesh_t>();
    entity_id_play = std::map<size_t, rgl_entity_t>();
    open_mmap(file_path_bin);
    yaml_root = YAML::LoadFile(file_path_yaml);
    for (size_t i = 0; yaml_root[FUNCTION_CALL + std::to_string(i)] ; ++i) {
        auto function_call = yaml_root[FUNCTION_CALL + std::to_string(i)];
        if (function_call[MESH_CREATE]) {
            play_mesh_create(function_call);
        } else if (function_call[MESH_DESTROY]) {
            play_mesh_destroy(function_call);
        } else if (function_call[MESH_UPDATE_VERTICES]) {
            play_mesh_update_vertices(function_call);
        } else if (function_call[ENTITY_CREATE]) {
            play_entity_create(function_call);
        } else if (function_call[ENTITY_DESTROY]) {
            play_entity_destroy(function_call);
        } else if (function_call[ENTITY_SET_POSE]) {
            play_entity_set_pose(function_call);
        } else exit(1);
    }
    munmap(file_mmap, mmap_size);
}

void Record::play_mesh_create(YAML::Node& mesh_create) {
    rgl_mesh_t mesh;
    rgl_mesh_create(&mesh,
                    reinterpret_cast<const rgl_vec3f*>(file_mmap + mesh_create["vertices"].as<size_t>()),
                    mesh_create["vertex_count"].as<int>(),
                    reinterpret_cast<const rgl_vec3i*>(file_mmap + mesh_create["indices"].as<size_t>()),
                    mesh_create["index_count"].as<int>());
    mesh_id_play.insert(std::make_pair(mesh_create["out_mesh"].as<size_t>(), mesh));
}

void Record::play_mesh_destroy(YAML::Node& mesh_destroy) {
    rgl_mesh_destroy(mesh_id_play[mesh_destroy["mesh"].as<size_t>()]);
}

void Record::play_mesh_update_vertices(YAML::Node& mesh_update_vertices) {
    rgl_mesh_update_vertices(mesh_id_play[mesh_update_vertices["mesh"].as<size_t>()],
                             reinterpret_cast<const rgl_vec3f*>
                             (file_mmap + mesh_update_vertices["vertices"].as<size_t>()),
                             mesh_update_vertices["vertex_count"].as<int>());
}

void Record::play_entity_create(YAML::Node& entity_create) {
    rgl_entity_t entity;
    rgl_entity_create(&entity, nullptr, mesh_id_play[entity_create["mesh"].as<size_t>()]);
    entity_id_play.insert(std::make_pair(entity_create["entity"].as<size_t>(), entity));
}

void Record::play_entity_destroy(YAML::Node& entity_destroy) {
    rgl_entity_destroy(entity_id_play[entity_destroy["entity"].as<size_t>()]);
}

void Record::play_entity_set_pose(YAML::Node& entity_set_pose) {
    rgl_entity_set_pose(entity_id_play[entity_set_pose["entity"].as<size_t>()],
                        reinterpret_cast<const rgl_mat3x4f*>
                        (file_mmap + entity_set_pose["local_to_world_tf"].as<size_t>()));
}
