#include <rgl/api/record_api.h>
void rgl_record_start(const char* file_path_yaml, const char* file_path_bin) {
    Record::instance().start(file_path_yaml, file_path_bin);
}
void rgl_record_stop() {
    Record::instance().stop();
}
void rgl_record_play(const char* file_path_yaml, const char* file_path_bin) {
    Record::instance().play(file_path_yaml, file_path_bin);
}

void rgl_record_rgl_mesh_create(rgl_mesh_t *out_mesh,
                                const rgl_vec3f *vertices,
                                int vertex_count,
                                const rgl_vec3i *indices,
                                int index_count) {
    Record::instance().record_mesh_create(out_mesh, vertices, vertex_count, indices, index_count);
}
void rgl_record_rgl_mesh_destroy(rgl_mesh_t mesh) {
    Record::instance().record_mesh_destroy(mesh);
}
void rgl_record_rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f *vertices, int vertex_count) {
    Record::instance().record_mesh_update_vertices(mesh, vertices, vertex_count);
}

void rgl_record_rgl_entity_create(rgl_entity_t *out_entity, rgl_scene_t scene, rgl_mesh_t mesh) {
    Record::instance().record_entity_create(out_entity, scene, mesh);
}
void rgl_record_rgl_entity_destroy(rgl_entity_t entity) {
    Record::instance().record_entity_destroy(entity);
}
void rgl_record_rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f *local_to_world_tf) {
    Record::instance().record_entity_set_pose(entity, local_to_world_tf);
}