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