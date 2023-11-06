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

#include <utility>

#include <APIObject.hpp>
#include <RGLFields.hpp>
#include <scene/Scene.hpp>
#include <scene/Mesh.hpp>
#include <math/Mat3x4f.hpp>

/**
 * Entity represents an object on a scene, consisting of:
 * - reference to mesh
 * - pose (local-to-world transform)
 * - (optional) reference to intensity texture
 * - (optional) id (for instance segmentation)
 * - etc.
 */
struct Entity : APIObject<Entity>
{
	friend struct APIObject<Entity>;
	friend struct Scene;

	/**
	 * Factory methods which creates an Entity and adds it to the given Scene.
	 * See constructor docs for more details.
	 */
	static std::shared_ptr<Entity> create(std::shared_ptr<Mesh> mesh);

	/**
	 * Sets ID that will be used as a point attribute ENTITY_ID_I32 when a ray hits this entity.
	 */
	void setId(int newId);

	/**
	 * Sets or updates Entity's transform.
	 */
	void setTransform(Mat3x4f newTransform);

	/**
	 * Sets intensity texture that will be used as a point attribute INTENSITY_F32 when a ray hits this entity.
	 */
	void setIntensityTexture(std::shared_ptr<Texture> texture);

	/**
	 * Returns Entity's transform such that it is possible to compute meaningful velocity between it and the current transform.
	 * Most often it will return the previous frame (if Entity is updated on each frame). See source for details.
	 * NOTE: It is assumed that (current) transform is always valid for the present scene time (even if it was set in the past).
	 * @return Mat3x4f of the local-to-world transform from the previous frame if available.
	 */
	std::optional<Mat3x4f> getPreviousFrameTransform() const;

private:
	/**
	 * Creates Entity with given mesh and identity transform.
	 * Before using Entity, it is required to register it on some Scene.
	 * However, it cannot be done without having its shared_ptr,
	 * therefore this constructor is private and Entity::create() should be used.
	 * @param mesh Mesh used by this Entity. May be shared by multiple Entities.
	 */
	Entity(std::shared_ptr<Mesh> mesh);

private:
	struct TransformWithTime
	{
		Mat3x4f matrix;
		std::optional<Time> time;
	};
	TransformWithTime transform{Mat3x4f::identity(), std::nullopt};
	TransformWithTime formerTransform{Mat3x4f::identity(), std::nullopt};

	Field<ENTITY_ID_I32>::type id{RGL_DEFAULT_ENTITY_ID};

	std::shared_ptr<Mesh> mesh{};
	std::shared_ptr<Texture> intensityTexture{};
};
