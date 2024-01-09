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
#include "Time.hpp"

/**
 * Proxy structure for reading TapeCall fields.
 * Expects a node of the following format:
 * fnName: {t: <int64_t>, a: [<arg0>, <arg1>, ...]}
 */
struct TapeCall
{
	std::string getFnName() const { return node.begin()->first.as<std::string>(); }
	Time getTimestamp() const { return Time::nanoseconds(node.begin()->second["t"].as<std::chrono::nanoseconds::rep>()); }
	YAML::Node getArgsNode() const { return node.begin()->second["a"]; }

	explicit TapeCall(const YAML::Node& node) : node(node)
	{
		if (!node.IsMap()) {
			throw std::invalid_argument("TapeCall node is not a map");
		}
	}

private: // FIELDS
	YAML::Node node;
};
