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

#include <scene/Mesh.hpp>
#include <scene/Scene.hpp>

API_OBJECT_INSTANCE(Mesh);

Mesh::Mesh(const Vec3f* vertices, size_t vertexCount, const Vec3i* indices, size_t indexCount)
{
	dVertices->copyFromExternal(vertices, vertexCount);
	dIndices->copyFromExternal(indices, indexCount);
}

void Mesh::setTexCoords(const Vec2f* texCoords, std::size_t texCoordCount)
{
	if (texCoordCount != dVertices->getCount()) {
		auto msg = fmt::format("Cannot set texture coordinates because vertex count do not match with "
		                       "texture coordinates count: vertexCount={}, textureCoords={}",
		                       dVertices->getCount(), texCoordCount);
		throw std::invalid_argument(msg);
	}

	if (!dTextureCoords.has_value()) {
		dTextureCoords = DeviceSyncArray<Vec2f>::create();
	}

	dTextureCoords.value()->copyFromExternal(texCoords, texCoordCount);
	Scene::instance().requestSBTRebuild();
}

void Mesh::setBoneWeights(const rgl_bone_weights_t* boneWeights, int32_t boneWeightsCount)
{
	if (boneWeightsCount != dVertices->getCount()) {
		auto msg = fmt::format("Cannot set bone weights because vertex count do not match with "
		                       "bone weights count: vertexCount={}, boneWeightsCount={}",
		                       dVertices->getCount(), boneWeightsCount);
		throw std::invalid_argument(msg);
	}

	if (!dBoneWeights.has_value()) {
		dBoneWeights = DeviceSyncArray<BoneWeights>::create();
	}

	dBoneWeights.value()->copyFromExternal(reinterpret_cast<const BoneWeights*>(boneWeights), boneWeightsCount);
}
