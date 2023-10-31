#pragma once

#include <vector>
#include <math.h>

template<typename T>
static std::vector<T> generateStaticColorTexture(int width, int height, T value)
{
	auto texels = std::vector<T>(width * height);

	for (int i = 0; i < width * height; ++i) {
		texels[i] = (T) value;
	}
	return texels;
}

template<typename T>
static std::vector<T> generateCheckerboardTexture(int width, int height)
{
	// Generate a sample texture with a grid pattern 16x16.
	int xGridSize = ceil(width / 16.0f);
	int yGridSize = ceil(height / 16.0f);
	int xStep = 0;
	int yStep = 0;

	auto texels = std::vector<T>(width * height);

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			texels[i * height + j] = (T) yStep * 0.5f + (T) xStep * 0.5f;
			if (j % yGridSize == 0) {
				yStep += yGridSize;
			}
		}
		yStep = 0;
		if (i % xGridSize == 0) {
			xStep += xGridSize;
		}
	}

	return texels;
}