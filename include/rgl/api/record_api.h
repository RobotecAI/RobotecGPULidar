#pragma once
#include <Record.h>

void rgl_record_start(const char* file_path_yaml, const char* file_path_bin);
void rgl_record_stop();
void rgl_record_play(const char* file_path_yaml, const char* file_path_bin);