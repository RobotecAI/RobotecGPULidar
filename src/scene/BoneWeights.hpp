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

#include <vector_types.h>

#include <rgl/api/core.h>

/**
* Same as `rgl_bone_weights_t` but uses vector types to speed up CUDA operations.
*/
struct BoneWeights
{
	float4 weights;
	int4 boneIndexes;
};
static_assert(sizeof(BoneWeights) == sizeof(rgl_bone_weights_t));
