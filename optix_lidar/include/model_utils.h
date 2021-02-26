#pragma once
#include "model.h"
#include <memory>
#include <string>

std::shared_ptr<Model> load_model_from_obj_file(const std::string & obj_file);
