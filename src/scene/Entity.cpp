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

#include <scene/Entity.hpp>

API_OBJECT_INSTANCE(Entity);

Entity::Entity(std::shared_ptr<Mesh> mesh, std::optional<std::string> name)
: mesh(std::move(mesh))
, transform(Mat3x4f::identity())
, humanReadableName(std::move(name)) { }

void Entity::setTransform(Mat3x4f newTransform)
{
	transform = newTransform;
	if (auto activeScene = scene.lock()) {
		activeScene->requestASRebuild();
	}
}

OptixInstance Entity::getIAS(int idx)
{
	// NOTE: this assumes a single SBT record per GAS
	OptixInstance instance = {
		.instanceId = static_cast<unsigned int>(idx),
		.sbtOffset = static_cast<unsigned int>(idx),
		.visibilityMask = 255,
		.flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT,
		.traversableHandle = mesh->getGAS(scene.lock()->getStream()),
	};
	transform.toRaw(instance.transform);
	return instance;
}
