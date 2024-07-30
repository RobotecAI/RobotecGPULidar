// Copyright 2024 Robotec.AI
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

#include <set>
#include <vector>
#include <optional>

#include <math/Mat3x4f.hpp>
#include <APIObject.hpp>

struct Bone
{
	std::optional<std::string> name{std::nullopt};
	std::set<int32_t> childIdxes{};
	Mat3x4f restposeToModelTransform{};
};

struct Skeleton : APIObject<Skeleton>
{
	friend APIObject<Skeleton>;

	const std::vector<Bone>& getBones() const { return bones; }

private:
	Skeleton(const int32_t* parentIdxes, const Mat3x4f* restposes, const char** names, std::size_t bonesCount);

private:
	std::vector<Bone> bones{};
};
