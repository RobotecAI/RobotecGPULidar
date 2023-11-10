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

std::shared_ptr<Entity> Entity::create(std::shared_ptr<Mesh> mesh)
{
	auto entity = APIObject<Entity>::create(mesh);
	Scene::instance()->addEntity(entity);
	return entity;
}

Entity::Entity(std::shared_ptr<Mesh> mesh) : mesh(std::move(mesh)) {}

void Entity::setTransform(Mat3x4f newTransform)
{
	transform = newTransform;
	Scene::instance()->requestASRebuild();
}

void Entity::setId(int newId)
{
	constexpr auto optixMaxInstanceId = 1 << 28; // https://raytracing-docs.nvidia.com/optix7/guide/index.html#limits#limits
	if (newId >= optixMaxInstanceId) {
		auto msg = fmt::format("Entity ID ({}) must be less than {}", newId, optixMaxInstanceId);
		throw std::invalid_argument(msg);
	}
	id = newId;
	Scene::instance()->requestASRebuild();
}

void Entity::setIntensityTexture(std::shared_ptr<Texture> texture)
{
	intensityTexture = texture;
	Scene::instance()->requestSBTRebuild();
}

Mat3x4f Entity::getVelocity() const { throw std::runtime_error("unimplemented"); }
