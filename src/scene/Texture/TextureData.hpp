


#include <texture_types.h>

struct TextureData {
public:


	TextureData();

	cudaTextureObject_t* GetTextureObject()  { return &textureObject; }


private:
	void* data{nullptr};
	cudaTextureObject_t textureObject;
	cudaArray_t dPixelArray;

};