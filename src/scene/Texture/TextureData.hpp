
#include <texture_types.h>
#include "../../RGLTextureFormats"
#include "math/Vector.hpp"


struct TextureData {
public:


	TextureData(void *data, rgl_texture_format format, Vec2i  resolution);

	~TextureData();

	cudaTextureObject_t GetTextureObject() { return textureObject; }


private:

	cudaChannelFormatDesc CreateChannelDescriptor(rgl_texture_format format);

	cudaTextureObject_t textureObject;
	cudaArray_t dPixelArray;

};