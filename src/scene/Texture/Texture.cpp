


#include <cstdint>
#include "Texture.hpp"


API_OBJECT_INSTANCE(Texture);

Texture::Texture(void *texels, rgl_texture_format format, int resolution, int id) :
		resolution(resolution, resolution),
		ID(id),
		format(format) {

	data = std::make_shared<TextureData>(texels, format, resolution);


}

Texture::~Texture() {


}



