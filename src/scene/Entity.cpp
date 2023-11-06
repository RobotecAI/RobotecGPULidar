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

std::shared_ptr<Entity> Entity::create(std::shared_ptr<Mesh> mesh, std::shared_ptr<Scene> scene)
{
	auto entity = APIObject<Entity>::create(mesh);
	scene->addEntity(entity);
	return entity;
}

Entity::Entity(std::shared_ptr<Mesh> mesh) : mesh(std::move(mesh)) {}

void Entity::setTransform(Mat3x4f newTransform)
{
	prevTransform = transform;
	transform = {newTransform, scene->getTime()};
	scene->requestFullRebuild();
}

void Entity::setId(int newId)
{
	constexpr auto optixMaxInstanceId = 1 << 28; // https://raytracing-docs.nvidia.com/optix7/guide/index.html#limits#limits
	if (newId >= optixMaxInstanceId) {
		auto msg = fmt::format("Entity ID ({}) must be less than {}", newId, optixMaxInstanceId);
		throw std::invalid_argument(msg);
	}
	id = newId;
	scene->requestASRebuild();
}

void Entity::setIntensityTexture(std::shared_ptr<Texture> texture)
{
	intensityTexture = texture;
	scene->requestSBTRebuild();
}

std::optional<Mat3x4f> Entity::getPrevFrameTransform() const
{
	// Currently, setting Scene time (rgl_scene_set_time) is optional.
	// Making it mandatory (refusing to raytrace without time set) would simplify the code below.
	// However, it would be a breaking change, requiring fixing all plugins.
	bool hasTransformsTimestamps = transform.time.has_value() && prevTransform.time.has_value();
	bool hasSceneTimestamps = scene->getTime().has_value() && scene->getPrevTime().has_value();
	if (!hasSceneTimestamps || !hasTransformsTimestamps) {
		return std::nullopt;
	}
	bool transformsTimestampsOk = transform.time.value() == scene->getTime().value() &&
	                              prevTransform.time.value() == scene->getPrevTime().value();
	if (!transformsTimestampsOk) {
		RGL_WARN("Detected stale transforms! Timestamps: entity=(curr={}, prev={}), scene=(curr={}, prev={})",
		         transform.time.value().asSeconds(), prevTransform.time.value().asSeconds(), scene->getTime()->asSeconds(),
		         scene->getPrevTime()->asSeconds());
		return std::nullopt;
	}
	return prevTransform.matrix;
}
