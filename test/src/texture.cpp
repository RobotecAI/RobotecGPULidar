#include <gtest/gtest.h>
#include "utils.hpp"


class TextureTest : public RGLTest {

protected:
	uint32_t* generateTexture(int size)
		{
			uint32_t* pixels = new uint32_t[size * size];
			for (int i = 0; i < size * size; i++) {
				pixels[i] = 0xffffffff;
			}
			return pixels;
		}
};

TEST_F(TextureTest, rgl_texture_create)
{
	auto textureData = generateTexture(256);
	rgl_texture_create(&textureData, 256, 256);


}