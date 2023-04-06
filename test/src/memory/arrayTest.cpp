#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <memory/InvalidArrayCast.hpp>
#include <memory/HostPinnedArray.hpp>

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

struct ArrayOps : public ::testing::Test
{
protected:
	using Type = int;
	ArrayOps() : array(HostPinnedArray<Type>::create()) { }
	HostPinnedArray<Type>::Ptr array;
};


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
