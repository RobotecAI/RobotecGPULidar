#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <memory/AbstractArrays.hpp>
#include <memory/ConcreteArrays.hpp>
#include <memory/InvalidArrayCast.hpp>

using namespace ::testing;

TEST(Array, Typing)
{
	using Type = int;
	using WrongType = float;

	// Create Array and its untyped handle.
	HostPinnedArray<Type>::Ptr arrayOriginal = HostPinnedArray<Type>::create();
	IAnyArray<MemoryKind::HostPinned>::ConstPtr arrayAny = arrayOriginal->asAnyArray();
	arrayOriginal->append(static_cast<Type>(42));

	// Attempt casting to a wrong Array type, should throw
	// Lambda because comma in explicit template parameter list fools GTEST macros
	auto cast = [&]() { arrayOriginal->asTypedArray<WrongType, HostPinnedArray>(); };
	EXPECT_THROW(cast(), InvalidArrayCast);

	// Casting to wrong memory kind is a compile error!
	// auto shouldNotCompile = arrayAny->asTypedArray<Type, DeviceSyncArray>();

	// Check casting to correct type works
	HostArray<MemoryKind::HostPinned, Type>::ConstPtr arrayOfTypeRecovered = arrayAny->asTypedArray<Type, HostPinnedArray>();
	EXPECT_THAT(arrayOfTypeRecovered, NotNull());
	EXPECT_THAT(arrayOfTypeRecovered, Eq(arrayOriginal));
	EXPECT_EQ(arrayOfTypeRecovered->at(0), static_cast<Type>(42));
}

// For some tests, we would like to wrap MemoryOperations (e.g. to count calls).
// To keep Array<M, T>::Array(MemoryOperations) constructor protected (as it should be),
// we define a test-only subclass that turns protected filed into a public one.
template<typename T>
struct HostArrayExposed : public HostPinnedArray<T>
{
	using Ptr = std::shared_ptr<HostArrayExposed<T>>;
	using HostArray<MemoryKind::HostPinned, T>::memOps; // Come here, cutie!

	static HostArrayExposed<T>::Ptr create()
	{
		return std::static_pointer_cast<HostArrayExposed<T>>(HostPinnedArray<T>::create());
	}
};

struct ArrayOps : public ::testing::Test
{
protected:
	using Type = int;
	using ArrayExposed = HostArrayExposed<Type>;
	struct MemOpsStats
	{
		int allocCount = {0};
		int deallocCount = {0};
		int copyCount = {0};
		int clearCount = {0};
	};

	ArrayOps()
	: array(std::static_pointer_cast<ArrayExposed>(ArrayExposed::create()))
	{
		hijack(array->memOps);
	}

	void hijack(MemoryOperations& source)
	{
		origMemOps = source;
		source = {
			.allocate = [&](size_t bytes) { stats.allocCount += 1; return origMemOps.allocate(bytes); },
			.deallocate = [&](void* ptr) { stats.deallocCount += 1; return origMemOps.deallocate(ptr); },
			.copy = [&](void* dst, const void* src, size_t bytes) { stats.copyCount += 1; return origMemOps.copy(dst, src, bytes); },
			.clear = [&](void* ptr, int value, size_t bytes) { stats.clearCount += 1; return origMemOps.clear(ptr, value, bytes); },
		};
	}

	ArrayExposed::Ptr array;
	MemOpsStats stats;
	MemoryOperations origMemOps;
};

TEST_F(ArrayOps, NoNaiveAppend)
{
	constexpr size_t END_SIZE = 16;
	for (int i = 0; i < END_SIZE; ++i) {
		array->append(1);
	}
	EXPECT_EQ(stats.allocCount, std::log2(END_SIZE) + 1);
	EXPECT_EQ(stats.deallocCount, std::log2(END_SIZE));
}

TEST_F(ArrayOps, ReservePreventsAllocs)
{
	constexpr size_t END_SIZE = 16;
	array->reserve(END_SIZE, false);
	for (int i = 0; i < END_SIZE; ++i) {
		array->append(1);
	}

	EXPECT_EQ(stats.allocCount, 1);
	EXPECT_EQ(stats.deallocCount, 0);
}

TEST_F(ArrayOps, ReserveDoesNotChangeCount)
{
	constexpr int END_SIZE = 42;
	EXPECT_EQ(array->getCount(), 0);
	EXPECT_EQ(array->getCapacity(), 0);
	EXPECT_THROW(array->at(0), std::out_of_range);
	array->reserve(END_SIZE, true);
	EXPECT_EQ(array->getCount(), 0);
	EXPECT_LE(array->getCapacity(), END_SIZE);
	EXPECT_THROW(array->at(0), std::out_of_range);
}

TEST_F(ArrayOps, ReservePreserveData)
{
	array->append(1);
	EXPECT_EQ(array->at(0), 1);
	array->reserve(4, true);
	EXPECT_EQ(array->at(0), 1);
	array->reserve(16, false);
	EXPECT_THROW(array->at(0), std::out_of_range);
}

TEST_F(ArrayOps, ResizeChangesCount)
{
	constexpr int END_SIZE = 42;
	EXPECT_EQ(array->getCount(), 0);
	EXPECT_EQ(array->getCapacity(), 0);
	EXPECT_THROW(array->at(0), std::out_of_range);
	array->resize(END_SIZE, true, false);
	EXPECT_EQ(array->at(END_SIZE-1), 0);
	EXPECT_EQ(array->getCount(), END_SIZE);
	EXPECT_LE(array->getCapacity(), END_SIZE);
}

// TODO(nebraszka): write more tests:
// TODO: resizing test
// TODO: copy test
// TODO: host operations test
