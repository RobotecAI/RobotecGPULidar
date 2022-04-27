#pragma once

#pragma once

#include <cuda.h>
#include <type_traits>
#include <optional>
#include "Logging.h"
#include "HostPinnedBuffer.hpp"
#include "data_types/PointTypes.h"
#include "utils/optix_macros.h"

#include <macros/cuda.hpp>

// TODO: elemCount and elemCapacity are duplictes..
template<typename T>
struct HostPinnedBuffer;

template <typename T>
struct DeviceBuffer
{
    typedef T ValueType;
private:
    static_assert(std::is_trivially_copyable<T>::value, "DeviceBuffer is instantiable only for types that can be copied between Host and GPU");

    T* data {nullptr};
    std::size_t elemCount {0};
    std::size_t elemCapacity {0};
    std::string _name;

public:
    DeviceBuffer(DeviceBuffer&) = delete;
    DeviceBuffer(DeviceBuffer&&) = delete;
    DeviceBuffer& operator=(DeviceBuffer&) = delete;
    DeviceBuffer& operator=(DeviceBuffer&&) = delete;

    // TODO: buffer names are most likely no longer needed - consider removing them
    DeviceBuffer(const std::string& name = "<unnamed>") : _name(name) {
        logInfo("[DB] DeviceBuffer(): {}\n", *this);
    }

    ~DeviceBuffer() {
        logInfo("[DB] ~DeviceBuffer(): {}\n", *this);
        if (data != nullptr) {
            cudaFree(data);
            data = nullptr;
        }
    }

    void copyFromHost(const T* src, std::size_t srcElemCount) {
        logInfo("[DB] copyFromHost({}, {}): {}\n", (void*) src, srcElemCount, *this);
        ensureDeviceCanFit(srcElemCount);
        CHECK_CUDA(cudaMemcpy(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice));
        elemCount = srcElemCount;
    }

    // TODO: deduplicate all those memcopies
    void copyFromHostAsync(const T* src, std::size_t srcElemCount, cudaStream_t stream) {
    	logInfo("[DB] copyFromHostAsync({}, {}, {}): {}\n", (void*) src, srcElemCount, (void*) stream, *this);
        ensureDeviceCanFit(srcElemCount);
        CHECK_CUDA(cudaMemcpyAsync(data, src, srcElemCount * sizeof(T), cudaMemcpyHostToDevice, stream));
        elemCount = srcElemCount;
    }

    void copyFromHost(const HostPinnedBuffer<int>& src)
    {
    	logInfo("[DB] copyFromHost(<hb>): {}\n", *this);
        copyFromHost(src.readHost(), src.getElemCount());
    }

    void copyFromHost(const std::vector<T>& src)
    {
        logInfo("[DB] copyFromHost(<vec>): {}\n", src.size(), *this);
        copyFromHost(src.data(), src.size()); }

    void copyPrefixToHostAsync(T* dst, size_t elemsToCopy, cudaStream_t stream)
    {
        logInfo("[DB] copyPrefixToHostAsync({}, {}, {}): {}\n", (void*) dst, elemsToCopy, (void*) stream, *this);
        if (elemsToCopy > elemCount) {
            auto msg = fmt::format("copyPrefixToHostAsync {} requested to copy more elements ({}) than it cointains ({})\n",
                                   _name, elemsToCopy, elemCount);
            throw std::invalid_argument(msg);
        }
        CHECK_CUDA(cudaMemcpyAsync(dst, data, elemsToCopy * sizeof(T), cudaMemcpyDeviceToHost, stream));
    }

    const T* readDevice() const {
        logInfo("[DB] readDevice(): {}\n", *this);
        return data;
    }

    T* writeDevice() {
        logInfo("[DB] writeDevice(): {}\n", *this);
        return data;
    }

    void print(int elemLimit=5) const {
        T* temp = nullptr;
        CHECK_CUDA(cudaMallocHost(reinterpret_cast<void**>(&temp), sizeof(T) * elemCount));
        CHECK_CUDA(cudaMemcpy(temp, data, sizeof(T) * elemCount, cudaMemcpyDeviceToHost));
        int toPrint = std::min((int) elemCount, elemLimit);

        // WTF: this print apparently flushes "[0:230400]:." in tests
        fmt::print("{}: ", _name);

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

        if (elemLimit < elemCount) {
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
        }

        fmt::print("\n");

        CHECK_CUDA(cudaFreeHost(temp));
    }

    CUdeviceptr readDeviceRaw() const {
        logInfo("[DB] readDeviceRaw(): {}\n", *this);
        return reinterpret_cast<CUdeviceptr>(
                reinterpret_cast<void*>(
                        const_cast<T*>(readDevice())
                )
        );
    }

    std::size_t getElemCount() const { return elemCount; }

    std::size_t getByteSize() const { return getElemCount() * sizeof(T); }

    void resizeToFit(std::size_t newElemCount, bool clear=false)
    {
    	logInfo("[DB] resizeToFit({}, {}): {}\n", newElemCount, clear, *this);
        ensureDeviceCanFit(newElemCount);
        elemCount = newElemCount;
        if (clear) {
            CHECK_CUDA(cudaMemset(data, 0, elemCount * sizeof(T)));
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
            CHECK_CUDA(cudaFree(data));
        }
        CHECK_CUDA(cudaMalloc(reinterpret_cast<void**>(&data), newElemCount * sizeof(T)));
        logInfo("[DB] ensureDeviceCanFit({}): {}\n", newElemCount, *this);
        elemCapacity = newElemCount;
    }

    friend struct fmt::formatter<DeviceBuffer<T>>;
};

#include <data_types/PointTypes.h>
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

template<typename T>
struct fmt::formatter<DeviceBuffer<T>>
{
	template<typename ParseContext>
	constexpr auto parse(ParseContext& ctx) {
		return ctx.begin();
	}

	template<typename FormatContext>
	auto format(DeviceBuffer<T> const& db, FormatContext& ctx)
	{
		return fmt::format_to(ctx.out(), "{} ({}/{}) @ {}", db._name, db.elemCount, db.elemCapacity, (void*) db.data);
	}
};
