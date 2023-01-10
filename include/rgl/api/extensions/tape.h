// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
 * Returns whether tape recording is active or not.
 * Currently, Windows is not supported: throws RGL_TAPE_ERROR
 * @param is_active address to store tape recording activation status
 */
RGL_API rgl_status_t rgl_tape_record_is_active(bool* is_active);

/**
 * Loads recorded API calls from files and exectues them.
 * Currently, Windows is not supported: throws RGL_TAPE_ERROR
 * @param path path to recording files (should contain filename without extension)
 */
RGL_API rgl_status_t rgl_tape_play(const char* path);