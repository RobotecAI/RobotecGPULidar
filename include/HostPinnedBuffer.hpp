#pragma once

#pragma once

#include <type_traits>
#include <optional>
#include "Logging.h"
#include "DeviceBuffer.hpp"
#include "utils/optix_macros.h"

template<typename T>
struct DeviceBuffer;

template <typename T>
struct HostPinnedBuffer
{
private:
    typedef T ValueType;
    static_assert(std::is_trivially_copyable<T>::value, "HostPinnedBuffer is instantiable only for types that can be copied between Host and GPU");

    T* data {nullptr};
    std::size_t elemCount {0};
    std::size_t elemCapacity {0};
    std::string name;

public:
    HostPinnedBuffer(HostPinnedBuffer&) = delete;
    HostPinnedBuffer(HostPinnedBuffer&&) = delete;
    HostPinnedBuffer& operator=(HostPinnedBuffer&) = delete;
    HostPinnedBuffer& operator=(HostPinnedBuffer&&) = delete;

    HostPinnedBuffer(const std::string& _name="<unnamed>") {
        logInfo("[DB] HostPinnedBuffer {} ({})\n", _name, elemCount);
        name = _name;
    }

    ~HostPinnedBuffer() {
        logInfo("[DB] ~HostPinnedBuffer {}\n", name);
        if (data != nullptr) {
            cudaFreeHost(data);
        }
    }

    void copyFromDeviceAsync(const DeviceBuffer<T>& src, cudaStream_t stream) {
        logInfo("[DB] copyFromDevice {} (srcCount={})\n", name, src.getElemCount());
        ensureHostCanFit(src.getElemCount());
        CUDA_CHECK(MemcpyAsync(data, src.readDevice(), src.getElemCount() * sizeof(T), cudaMemcpyDeviceToHost, stream));
        elemCount = src.getElemCount();
    }

    const T* readHost() const {
        logInfo("[DB] readHost {}\n", name);
        return data;
    }

    T* writeHost() {
        logInfo("[DB] writeHost {}\n", name);
        return data;
    }


    std::size_t getElemCount() const {
        return elemCount;
    }

    std::size_t getByteSize() const {
        return getElemCount() * sizeof(T);
    }

    void resizeToFit(std::size_t newElemCount, bool clear=false)
    {
        ensureHostCanFit(newElemCount);
        elemCount = newElemCount;
        if (clear) {
            CUDA_CHECK(Memset(data, 0, elemCount * sizeof(T)));
        }
    }

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
