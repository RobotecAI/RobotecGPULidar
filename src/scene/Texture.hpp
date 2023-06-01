// Copyright 2023 Robotec.AI
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
#include <rgl/api/core.h>

struct Texture : APIObject<Texture> {

public:

	~Texture();

	Vec2i getResolution() const { return resolution; }

	size_t getWidth() const { return resolution.x(); }

	size_t getHeight() const { return resolution.y(); }

	cudaTextureObject_t GetTextureObject() const { return dTextureObject; }


private:
	// TODO (prybicki) Should I pollute internal class with api enum?
	Texture(const void* texels, int width, int height);

	Texture(const Texture&) = delete; // non construction-copyable
	Texture &operator=(const Texture&) = delete; // non copyable

	void createTextureObject(const void *texels, int width, int height);

	//cudaChannelFormatDesc CreateChannelDescriptor(rgl_texture_format_t format);

	friend APIObject<Texture>;
	Vec2i resolution{-1};

	cudaTextureObject_t dTextureObject;
	cudaArray_t dPixelArray;
};