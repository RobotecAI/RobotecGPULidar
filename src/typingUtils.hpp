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

#include <typeindex>
#include <string>

static std::string name(const std::type_index& typeIndex)
{
	std::string_view name = typeIndex.name();
	name.remove_prefix(name.find_first_not_of("0123456789"));
	return std::string(name);
}

static std::string name(const std::type_info& typeInfo) { return name(std::type_index(typeInfo)); }
