#include <gtest/gtest.h>
#include "utils.hpp"


class TextureTest : public RGLTest {

protected:
	template <typename T>
		static T* generateTexture(int size)
		{
			T* texels = new T[size * size];
			for (int i = 0; i < size * size; i++) {
				texels[i] = 0.1f;
			}
			return texels;
		}
};

TEST_F(TextureTest, rgl_texture_create)
{
	rgl_configure_logging(RGL_LOG_LEVEL_ALL, nullptr, true);
	auto textureRawData = generateTexture<float>(256);
	rgl_texture_t texture;

	auto temp = textureRawData[0];
	EXPECT_RGL_SUCCESS(rgl_texture_create(&texture,textureRawData, RGL_TEXTURE_TYPE_FLOAT,  256, 256, 1));
}