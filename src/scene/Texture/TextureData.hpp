
#include <texture_types.h>
#include "../../RGLTextureFormats"


struct TextureData {
public:


	TextureData(void *data, rgl_texture_format format, int resolution);

	~TextureData();

	cudaTextureObject_t *GetTextureObject() { return &textureObject; }


private:
	cudaTextureObject_t textureObject;
	cudaArray_t dPixelArray;

};