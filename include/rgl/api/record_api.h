#pragma once
#include <Record.h>

void rgl_record_start(const char* file_path_yaml, const char* file_path_bin);
void rgl_record_stop();
void rgl_record_play(const char* file_path_yaml, const char* file_path_bin);

void rgl_record_rgl_mesh_create(rgl_mesh_t *out_mesh,
                        const rgl_vec3f *vertices,
                        int vertex_count,
                        const rgl_vec3i *indices,
                        int index_count);
void rgl_record_rgl_mesh_destroy(rgl_mesh_t mesh);
void rgl_record_rgl_mesh_update_vertices(rgl_mesh_t mesh, const rgl_vec3f *vertices, int vertex_count);

void rgl_record_rgl_entity_create(rgl_entity_t *out_entity, rgl_scene_t scene, rgl_mesh_t mesh);
void rgl_record_rgl_entity_destroy(rgl_entity_t entity);
void rgl_record_rgl_entity_set_pose(rgl_entity_t entity, const rgl_mat3x4f *local_to_world_tf);