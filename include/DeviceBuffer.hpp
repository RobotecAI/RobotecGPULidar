#pragma once

#pragma once

#include <type_traits>
#include <optional>
#include "Logging.h"

template <typename T>
struct DeviceBuffer
{
    typedef T ValueType;
    static_assert(std::is_trivially_copyable<T>::value, "DeviceBuffer is instantiable only for types that can be copied between Host and GPU");

    T* data {nullptr};
    std::size_t elemCount {0};
    std::size_t elemCapacity {0};
    std::string name;

public:
    DeviceBuffer(DeviceBuffer&) = delete;
    DeviceBuffer(DeviceBuffer&&) = delete;
    DeviceBuffer& operator=(DeviceBuffer&) = delete;
    DeviceBuffer& operator=(DeviceBuffer&&) = delete;


    DeviceBuffer(const std::string& _name) {
        logInfo("[DB] DeviceBuffer {} ({})\n", _name, elemCount);
        name = _name;
    }

    ~DeviceBuffer() {
        logInfo("[DB] ~DeviceBuffer {}\n", name);
        if (data != nullptr) {
            cudaFree(data);
        }
    }

    void copyFromHost(const T* src, std::size_t srcElemCount) {
        logInfo("[DB] copyFromHost {} (count={})\n", name, srcElemCount);
        ensureDeviceCanFit(srcElemCount);
        CUDA_CHECK(Memcpy(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice));
        elemCount = srcElemCount;
    }

    void copyFromHost(const std::vector<T>& src) { copyFromHost(src.data(), src.size()); }

    const T* readDevice() const {
        logInfo("[DB] readDevice {}\n", name);
        return data;
    }

    T* writeDevice() {
        logInfo("[DB] writeDevice {}\n", name);
        return data;
    }

    CUdeviceptr readDeviceRaw() const {
        logInfo("[DB] readDeviceRaw {}\n", name);
        return reinterpret_cast<CUdeviceptr>(
                reinterpret_cast<void*>(
                        const_cast<T*>(readDevice())
                        )
                        );
    }

    std::size_t getElemCount() const {
        return elemCount;
    }

    std::size_t getByteSize() const {
        return getElemCount() * sizeof(T);
    }

    void resizeToFit(std::size_t newElemCount, bool clear=false)
    {
        ensureDeviceCanFit(newElemCount);
        elemCount = newElemCount;
        if (clear) {
            CUDA_CHECK(Memset(data, 0, elemCount * sizeof(T)));
        }
    }

private:
    void ensureDeviceCanFit(std::size_t newElemCount) {
        if (newElemCount == 0) {
            auto msg = fmt::format("Attempted to allocate {} bytes of memory\n", newElemCount);
            throw std::logic_error(msg);
        }
        if (elemCapacity >= newElemCount) {
            return;
        }
        if (data != nullptr) {
            CUDA_CHECK(Free(data));
        }
        CUDA_CHECK(Malloc(&data, newElemCount * sizeof(T)));
        logInfo("[DB] ensureDeviceCanFit {} {} {}\n", name, newElemCount, (void*) data);
        elemCapacity = newElemCount;
    }
};
