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

#include <math/Vector.hpp>
#include <memory/Array.hpp>
#include <scene/Scene.hpp>

struct ExternalAnimator
{
	friend struct Entity;

	explicit ExternalAnimator(DeviceSyncArray<Vec3f>::Ptr originalVertices);
	void animate(const Vec3f* vertices, std::size_t vertexCount);

private:
	DeviceSyncArray<Vec3f>::Ptr dAnimatedVertices = DeviceSyncArray<Vec3f>::create();
	DeviceSyncArray<Vec3f>::Ptr dVertexAnimationDisplacement = DeviceSyncArray<Vec3f>::create();
};
