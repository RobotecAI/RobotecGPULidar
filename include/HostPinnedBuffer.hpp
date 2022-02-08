#pragma once

#pragma once

#include <type_traits>
#include <optional>
#include "Logging.h"


template <typename T>
struct HostPinnedBuffer
{
    typedef T ValueType;
    static_assert(std::is_trivially_copyable<T>::value, "HostPinnedBuffer is instantiable only for types that can be copied between Host and GPU");

    T* data {nullptr};
    std::size_t elemCount {0};
    std::size_t elemCapacity {0};
    std::string name;

    HostPinnedBuffer(HostPinnedBuffer&) = delete;
    HostPinnedBuffer(HostPinnedBuffer&&) = delete;
    HostPinnedBuffer& operator=(HostPinnedBuffer&) = delete;
    HostPinnedBuffer& operator=(HostPinnedBuffer&&) = delete;

    HostPinnedBuffer(const std::string& _name) {
        logInfo("[DB] HostPinnedBuffer {} ({})\n", _name, elemCount);
        name = _name;
    }

    ~HostPinnedBuffer() {
        logInfo("[DB] ~HostPinnedBuffer {}\n", name);
        if (data != nullptr) {
            cudaFreeHost(data);
        }
    }

    void copyFromDeviceAsync(const DeviceBuffer<T>& src) {
        logInfo("[DB] copyFromDevice {} (from={}) (count={})\n", name, src.name,  src.getElemCount());
        ensureHostCanFit(src.getElemCount());
        CUDA_CHECK(MemcpyAsync(data, src.readDevice(), src.getElemCount() * sizeof(T), cudaMemcpyDeviceToHost));
        elemCount = src.getElemCount();
    }

    const T* readHost() {
        logInfo("[DB] readHost {}\n", name);
        return data;
    }

    // T* writeHost() {
    //     logInfo("[DB] writeHost {}\n", name);
    //     return data;
    // }


    std::size_t getElemCount() {
        return elemCount;
    }

    std::size_t getByteSize() {
        return getElemCount() * sizeof(T);
    }

private:
    void ensureHostCanFit(std::size_t newElemCount) {
        if (newElemCount == 0) {
            auto msg = fmt::format("Attempted to allocate {} bytes of memory\n", newElemCount);
            throw std::logic_error(msg);
        }
        if (elemCapacity >= newElemCount) {
            return;
        }
        if (data != nullptr) {
            CUDA_CHECK(FreeHost(data));
        }
        CUDA_CHECK(MallocHost(&data, newElemCount * sizeof(T)));
        elemCapacity = newElemCount;
    }
};
