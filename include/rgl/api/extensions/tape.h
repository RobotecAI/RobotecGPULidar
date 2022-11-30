#pragma once

#include <rgl/api/core.h>

/**
 * Starts recording all API calls.
 * Two files will be created at the path location: .yaml and .bin file.
 * Only one record session can be executed at the same time.
 * Currently, Windows is not supported: throws RGL_TAPE_ERROR
 * @param path path to output files (should contain filename without extension)
 */
RGL_API rgl_status_t rgl_tape_record_begin(const char* path);

/**
 * Stops active recording session and saves the recorded data to files (path determined at recording start)
 * Currently, Windows is not supported: throws RGL_TAPE_ERROR
 */
RGL_API rgl_status_t rgl_tape_record_end();

/**
 * Loads recorded API calls from files and exectues them.
 * Currently, Windows is not supported: throws RGL_TAPE_ERROR
 * @param path path to recording files (should contain filename without extension)
 */
RGL_API rgl_status_t rgl_tape_play(const char* path);