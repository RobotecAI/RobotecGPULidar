#pragma once

/**
 * two files will be created at path location: a .yaml and .bin file,
 * only meshes, entities and their poses are recorded (no lidar data)
 */
RGL_API rgl_status_t rgl_tape_record_begin(const char* path);

/**
 * saves the recorded data to two files: a .yaml and .bin file (path determined at recording start)
 */
RGL_API rgl_status_t rgl_tape_record_end();

/**
 * two files are required at path location: a .yaml and .bin file
 */
RGL_API rgl_status_t rgl_tape_play(const char* path);