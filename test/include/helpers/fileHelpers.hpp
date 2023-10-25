#pragma once

#include <helpers/commonHelpers.hpp>

#include <vector>
#include <filesystem>
#include <fstream>

template<typename T>
static std::vector<T> loadVec(std::filesystem::path path)
{
	// open the file:
	std::streampos fileSize;
	std::ifstream file(path, std::ios::binary);

	EXPECT_TRUE(file.is_open() && !file.eof());

	// get its size:
	file.seekg(0, std::ios::end);
	fileSize = file.tellg();
	file.seekg(0, std::ios::beg);

	EXPECT_TRUE(fileSize % sizeof(T) == 0);

	// read the data:
	std::vector<T> fileData(fileSize / sizeof(T));
	file.read((char*) &fileData[0], fileSize);
	return fileData;
}

static std::string readFileStr(std::filesystem::path path)
{
	std::vector<char> logFileChars = loadVec<char>(path);
	return {logFileChars.begin(), logFileChars.end()};
}

static rgl_mesh_t loadMesh(std::filesystem::path path)
{
	rgl_mesh_t mesh = nullptr;
	std::vector<rgl_vec3f> vs = loadVec<rgl_vec3f>(path.string() + std::string(".vertices"));
	std::vector<rgl_vec3i> is = loadVec<rgl_vec3i>(path.string() + std::string(".indices"));
	EXPECT_RGL_SUCCESS(rgl_mesh_create(&mesh, vs.data(), vs.size(), is.data(), is.size()));
	return mesh;
}