

#include "TextureData.hpp"
#include "macros/cuda.hpp"
#include <cuda_runtime.h>

TextureData::TextureData(void *data, rgl_texture_format format, Vec2i  resolution) {
	cudaResourceDesc res_desc = {};

	int32_t width = resolution.x();
	int32_t height = resolution.y();
	int32_t numComponents = 1;

	//cudaChannelFormatDesc channel_desc = CreateChannelDescriptor(format);
	cudaChannelFormatDesc channel_desc = cudaCreateChannelDesc<float1>();

	int32_t pitch = width * numComponents * getFormatSize(format);

	// TODO prybicki
	// Should we leave it like this, or add new copiers in DeivceBuffer.hpp?
	// Current copyFromHost and ensureDeviceCanFit are not working with cudaArray_t
	CHECK_CUDA(cudaMallocArray(&dPixelArray, &channel_desc, width, height));

	CHECK_CUDA(cudaMemcpy2DToArray(
			dPixelArray,
			0, 0,
			data,
			pitch, pitch, height,
			cudaMemcpyHostToDevice));

	res_desc.resType = cudaResourceTypeArray;
	res_desc.res.array.array = dPixelArray;

	cudaTextureDesc tex_desc = {};

	tex_desc.addressMode[0] = cudaAddressModeWrap;
	tex_desc.addressMode[1] = cudaAddressModeWrap;
	tex_desc.filterMode = cudaFilterModeLinear;
	tex_desc.readMode = cudaReadModeElementType;
	tex_desc.normalizedCoords = 1;
	tex_desc.maxAnisotropy = 1;
	tex_desc.maxMipmapLevelClamp = 99;
	tex_desc.minMipmapLevelClamp = 0;
	tex_desc.mipmapFilterMode = cudaFilterModePoint;
	tex_desc.borderColor[0] = 1.0f;
	// TO check tex_desc.sRGB = 0;

	CHECK_CUDA(cudaCreateTextureObject(&textureObject, &res_desc, &tex_desc, nullptr));

}

TextureData::~TextureData() {
	CHECK_CUDA(cudaDestroyTextureObject(textureObject));
	CHECK_CUDA(cudaFreeArray(dPixelArray));
}

cudaChannelFormatDesc TextureData::CreateChannelDescriptor(rgl_texture_format format) {
	cudaChannelFormatDesc channel_desc;

	switch (format) {

		case RGL_TEXTURE_TYPE_INT:
			return cudaCreateChannelDesc<int1>();
		case RGL_TEXTURE_TYPE_FLOAT:
			return cudaCreateChannelDesc<float1>();
	}

}
