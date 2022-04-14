#pragma once

#pragma once

#include <cuda.h>
#include <type_traits>
#include <optional>
#include "Logging.h"
#include "HostPinnedBuffer.hpp"
#include "data_types/PointTypes.h"
#include "utils/optix_macros.h"

// TODO: elemCount and elemCapacity are duplictes..
template<typename T>
struct HostPinnedBuffer;

template <typename T>
struct DeviceBuffer
{
private:
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


    // TODO: buffer names are most likely no longer needed - consider removing them
    DeviceBuffer(const std::string& _name = "<unnamed>") {
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

    // TODO: deduplicate all those memcopies
    void copyFromHostAsync(const T* src, std::size_t srcElemCount, cudaStream_t stream) {
        logInfo("[DB] copyFromHost {} (count={})\n", name, srcElemCount);
        ensureDeviceCanFit(srcElemCount);
        CUDA_CHECK(MemcpyAsync(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice, stream));
        elemCount = srcElemCount;
    }

    void copyFromHost(const HostPinnedBuffer<int>& src) {
        copyFromHost(src.readHost(), src.getElemCount());
    }

    void copyFromHost(const std::vector<T>& src) { copyFromHost(src.data(), src.size()); }

    void copyPrefixToHostAsync(T* dst, size_t elemsToCopy, cudaStream_t stream) {
        if (elemsToCopy > elemCount) {
            auto msg = fmt::format("copyPrefixToHostAsync {} requested to copy more elements ({}) than it cointains ({})\n",
                                   name, elemsToCopy, elemCount);
            throw std::invalid_argument(msg);
        }
        CUDA_CHECK(MemcpyAsync(dst, data, elemsToCopy * sizeof(T), cudaMemcpyDeviceToHost, stream));
    }

    const T* readDevice() const {
        logInfo("[DB] readDevice {}\n", name);
        return data;
    }

    T* writeDevice() {
        logInfo("[DB] writeDevice {}\n", name);
        return data;
    }

    void print(int elemLimit=5) const {
        T* temp = nullptr;
        CUDA_CHECK(MallocHost(&temp, sizeof(T) * elemCount));
        CUDA_CHECK(Memcpy(temp, data, sizeof(T) * elemCount, cudaMemcpyDeviceToHost));
        int toPrint = std::min((int) elemCount, elemLimit);

        // WTF: this print apparently flushes "[0:230400]:." in tests
        fmt::print("\n");

        // Prefix
        fmt::print("[");
        for (int i = 0; i < toPrint; ++i) {
            fmt::print("{}", temp[i]);
            if (i != toPrint - 1) {
                fmt::print(", ");
            }
        }
        if (toPrint < elemCount) {
            fmt::print(", ...");
        }
        fmt::print("]");

        fmt::print(" (total: {}) ", elemCount);

        // Suffix
        fmt::print("[");
        if (toPrint < elemCount) {
            fmt::print("..., ");
        }
        for (int i = elemCount - toPrint; i < elemCount; ++i) {
            fmt::print("{}", temp[i]);
            if (i != elemCount - 1) {
                fmt::print(", ");
            }
        }
        fmt::print("]");

        fmt::print("\n");

        CUDA_CHECK(FreeHost(temp));
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

template<>
struct fmt::formatter<Point3f>
{
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx) { return ctx.begin(); }

    template<typename FormatContext>
    auto format(Point3f const& p, FormatContext& ctx) {
        return fmt::format_to(ctx.out(), "({0} {1} {2})", p.x, p.y, p.z);
    }
};