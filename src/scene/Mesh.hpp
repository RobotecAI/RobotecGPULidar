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

#include <APIObject.hpp>
#include <math/Vector.hpp>
#include <memory/Array.hpp>

/**
 * Represents mesh data (at the moment vertices and indices) stored on the GPU.
 * Mesh, on its own, is not bound to any scene and can be used for different scenes.
 */
struct Mesh : APIObject<Mesh>
{
	friend APIObject<Mesh>;
	friend struct Scene;
	friend struct Entity;

	/**
	 * Sets textures coordinates to the mesh. Vertex count and texture coordinates count must be equal.
	 * After this operation, GAS needs to be rebuilt. This is handled internally in getGAS.
	 */
	void setTexCoords(const Vec2f* texCoords, std::size_t texCoordCount);

private:
	Mesh(const Vec3f* vertices, std::size_t vertexCount, const Vec3i* indices, std::size_t indexCount);

private:
	DeviceSyncArray<Vec3f>::Ptr dVertices = DeviceSyncArray<Vec3f>::create();
	DeviceSyncArray<Vec3i>::Ptr dIndices = DeviceSyncArray<Vec3i>::create();
	std::optional<DeviceSyncArray<Vec2f>::Ptr> dTextureCoords;
};
