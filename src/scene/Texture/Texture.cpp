


#include <cstdint>
#include "Texture.hpp"


API_OBJECT_INSTANCE(Texture);

Texture::Texture(void *texels, rgl_texture_format format, int width, int height, int id) :
		resolution(width, height),
		ID(id),
		format(format) {

	data = std::make_shared<TextureData>(texels, format, resolution);


}

Texture::~Texture() {


}



