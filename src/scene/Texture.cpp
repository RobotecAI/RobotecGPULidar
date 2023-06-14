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

#include <scene/Texture.hpp>
#include <cuda_runtime.h>

API_OBJECT_INSTANCE(Texture);

Texture::Texture(const void* texels, int width, int height) try :
		resolution(width, height)
		{
			createTextureObject(texels, width, height);
		}
		catch (const std::exception& e)
		{
			cleanup();
		}

void Texture::createTextureObject(const void* texels, int width, int height)
{
	cudaResourceDesc res_desc = {};

	int32_t numComponents = 1;

	cudaChannelFormatDesc channel_desc = cudaCreateChannelDesc<char>();

	int32_t pitch = width * numComponents * sizeof(char);

	// TODO prybicki
	// Should we leave it like this, or add new copiers in DeivceBuffer.hpp?
	// Current copyFromHost and ensureDeviceCanFit are not working with cudaArray_t
	CHECK_CUDA(cudaMallocArray(&dPixelArray, &channel_desc, width, height));

	CHECK_CUDA(cudaMemcpy2DToArray(
			dPixelArray,
			0, 0,
			texels,
			pitch, pitch, height,
			cudaMemcpyHostToDevice));

	res_desc.resType = cudaResourceTypeArray;
	res_desc.res.array.array = dPixelArray;

	cudaTextureDesc tex_desc = {};

	tex_desc.addressMode[0] = cudaAddressModeWrap;
	tex_desc.addressMode[1] = cudaAddressModeWrap;
	tex_desc.filterMode = cudaFilterModePoint;
	tex_desc.readMode = cudaReadModeElementType;
	tex_desc.normalizedCoords = 1;
	tex_desc.maxAnisotropy = 1;
	tex_desc.maxMipmapLevelClamp = 99;
	tex_desc.minMipmapLevelClamp = 0;
	tex_desc.mipmapFilterMode = cudaFilterModePoint;
	tex_desc.borderColor[0] = 1.0f;

	CHECK_CUDA(cudaCreateTextureObject(&dTextureObject, &res_desc, &tex_desc, nullptr));
}

Texture::~Texture()
{
	cleanup();
}

void Texture::cleanup()
{
	cudaDestroyTextureObject(dTextureObject);
	cudaFreeArray(dPixelArray);

	dTextureObject = 0;
	dPixelArray = nullptr;
}
