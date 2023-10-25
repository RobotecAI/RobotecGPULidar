#include <rgl/api/core.h>
#include <spdlog/fmt/fmt.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

typedef rgl_status_t (*rgl_get_version_t)(int32_t* out_major, int32_t* out_minor, int32_t* out_patch);
typedef rgl_status_t (*rgl_get_extension_info_t)(rgl_extension_t extension, int32_t* out_available);


// RAII Wrapper for dl-opening RGL to get version and extensions.
#ifdef _WIN32
struct MicroRGL
{
	MicroRGL(const char* path)
	{
		handle = LoadLibraryA(path);
		if (handle == NULL) {
			std::string hint;
			if (GetLastError() == 126) {
				hint = "Make sure that all dependencies of the library can be found.";
			}
			auto msg = fmt::format("LoadLibrary error: {} {}\n", GetLastError(), hint);
			throw std::runtime_error(msg);
		}
	}

	~MicroRGL()
	{
		if (handle != NULL) {
			FreeLibrary(handle);
		}
	}

	rgl_get_version_t get_rgl_get_version()
	{
		auto dyn_rgl_get_version = reinterpret_cast<rgl_get_version_t>(GetProcAddress(handle, "rgl_get_version_info"));
		if (dyn_rgl_get_version == NULL) {
			auto msg = fmt::format("GetProcAddress error: {}\n", GetLastError());
			throw std::runtime_error(msg);
		}
		return dyn_rgl_get_version;
	}

	rgl_get_extension_info_t get_rgl_get_extension_infos()
	{
		auto dyn_rgl_get_extension_info = reinterpret_cast<rgl_get_extension_info_t>(
		    GetProcAddress(handle, "rgl_get_extension_info"));
		if (dyn_rgl_get_extension_info == NULL) {
			auto msg = fmt::format("GetProcAddress error: {}\n", GetLastError());
			throw std::runtime_error(msg);
		}
		return dyn_rgl_get_extension_info;
	}

private:
	HMODULE handle = NULL;
};
#else
struct MicroRGL
{
	MicroRGL(const char* path)
	{
		handle = dlopen(path, RTLD_LAZY);
		if (handle == nullptr) {
			auto msg = fmt::format("dlopen error: {}\n", dlerror());
			throw std::runtime_error(msg);
		}
	}

	~MicroRGL()
	{
		if (handle != nullptr) {
			dlclose(handle);
		}
	}

	rgl_get_version_t get_rgl_get_version()
	{
		auto dyn_rgl_get_version = reinterpret_cast<rgl_get_version_t>(dlsym(handle, "rgl_get_version_info"));
		if (dyn_rgl_get_version == nullptr) {
			auto msg = fmt::format("dlsym error: {}\n", dlerror());
			throw std::runtime_error(msg);
		}
		return dyn_rgl_get_version;
	}

	rgl_get_extension_info_t get_rgl_get_extension_infos()
	{
		auto dyn_rgl_get_extension_info = reinterpret_cast<rgl_get_extension_info_t>(dlsym(handle, "rgl_get_extension_info"));
		if (dyn_rgl_get_extension_info == nullptr) {
			auto msg = fmt::format("dlsym error: {}\n", dlerror());
			throw std::runtime_error(msg);
		}
		return dyn_rgl_get_extension_info;
	};


private:
	void* handle = nullptr;
};
#endif


int main(int argc, char** argv)
try {
	if (argc != 2) {
		fmt::print(stderr, "Usage: {} <libRGL_path>\n", argv[0]);
		return 1;
	}

	MicroRGL lib{argv[1]};

	int32_t major = 0, minor = 0, patch = 0;
	rgl_status_t status = lib.get_rgl_get_version()(&major, &minor, &patch);
	fmt::print("rgl_get_version(...) -> {}.{}.{} [status={}]\n", major, minor, patch, status);

	int32_t extensionID = 0;
	while (status == RGL_SUCCESS) {
		int32_t available = -1;
		status = lib.get_rgl_get_extension_infos()(static_cast<rgl_extension_t>(extensionID), &available);
		fmt::print("rgl_get_extension_info({}, ...) -> {} [status={}]\n", extensionID, available, status);
		extensionID += 1;
	}
	return 0;
}
catch (std::runtime_error& e) {
	fmt::print(stderr, "{}\n", e.what());
}
