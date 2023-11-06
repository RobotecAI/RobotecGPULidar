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
	Scene::instance().addEntity(entity);
	return entity;
}

Entity::Entity(std::shared_ptr<Mesh> mesh) : mesh(std::move(mesh)) {}

void Entity::setTransform(Mat3x4f newTransform)
{
	formerTransform = transform;
	transform = {newTransform, Scene::instance().getTime()};
	Scene::instance().requestASRebuild(); // Current transform
	Scene::instance().requestSBTRebuild(); // Previous transform
}

void Entity::setId(int newId)
{
	constexpr auto optixMaxInstanceId = 1 << 28; // https://raytracing-docs.nvidia.com/optix7/guide/index.html#limits#limits
	if (newId >= optixMaxInstanceId) {
		auto msg = fmt::format("Entity ID ({}) must be less than {}", newId, optixMaxInstanceId);
		throw std::invalid_argument(msg);
	}
	id = newId;
	Scene::instance().requestASRebuild(); // Update instanceId field in AS
}

void Entity::setIntensityTexture(std::shared_ptr<Texture> texture)
{
	intensityTexture = texture;
	Scene::instance().requestSBTRebuild();
}

std::optional<Mat3x4f> Entity::getPreviousFrameTransform() const
{
	// At the moment of writing, setting Scene time (rgl_scene_set_time) is optional.
	// Making it mandatory (e.g. refusing to raytrace without time set) would simplify the code below.
	// However, it would be a breaking change, requiring fixing all plugins.

	bool transformWasSetOnlyOnce = transform.time.has_value() && !formerTransform.time.has_value();
	if (transformWasSetOnlyOnce) {
		// The first transform set might be the last one (e.g. static objects),
		// so let's assume this object was in the same pose in the previous frame to get zero velocity.
		return transform.matrix;
	}

	bool formerTransformWasSetInPrecedingFrame = formerTransform.time.has_value() &&
	                                             formerTransform.time == Scene::instance().getPrevTime();
	if (!formerTransformWasSetInPrecedingFrame) {
		return std::nullopt;
	}

	return formerTransform.matrix;
}
