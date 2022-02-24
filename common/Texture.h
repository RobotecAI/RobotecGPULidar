#pragma once

#include "gdt/gdt/math/vec.h"

struct Texture {
    Texture(int id)
        : texture_id(std::to_string(id))
    {
    }

    std::string texture_id;
    ~Texture()
    {
        if (pixel)
            delete[] pixel;
    }

    uint32_t* pixel { nullptr };
    gdt::vec2i resolution { -1 };

    Texture(const Texture&) = delete; // non construction-copyable
    Texture& operator=(const Texture&) = delete; // non copyable
};
