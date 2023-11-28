// Copyright 2023 Robotec.AI
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

#include <yaml-cpp/yaml.h>

#include <Logger.hpp>

// Type used as a key in TapePlayer object registry
using TapeAPIObjectID = size_t;

struct PlaybackState
{
	void clear();

	template<typename T>
	T* getPtr(const YAML::Node& offsetYamlNode)
	{
		assert(fileMmap != nullptr);
		auto offset = offsetYamlNode.as<size_t>();
		if (offset > mmapSize) {
			throw std::runtime_error(fmt::format("Tape binary offset ({}) out of range ({})", offset, mmapSize));
		}
		return reinterpret_cast<T*>(fileMmap + offset);
	}

	std::unordered_map<TapeAPIObjectID, rgl_mesh_t> meshes;
	std::unordered_map<TapeAPIObjectID, rgl_entity_t> entities;
	std::unordered_map<TapeAPIObjectID, rgl_texture_t> textures;
	std::unordered_map<TapeAPIObjectID, rgl_node_t> nodes;

private:
	uint8_t* fileMmap{nullptr};
	size_t mmapSize{0};

	friend struct TapePlayer; // To access private members
};
