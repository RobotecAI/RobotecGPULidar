#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <memory/AbstractArrays.hpp>
#include <memory/ConcreteArrays.hpp>
#include <memory/InvalidArrayCast.hpp>
#include <memory/DeviceAsyncArrayManager.hpp>

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

TEST(Array, ChangeStream)
{
	auto streamA = CudaStream::create();
	auto streamB = CudaStream::create();
	DeviceAsyncArray<int>::Ptr array = DeviceAsyncArray<int>::create(streamA);
	array->resize(5, true, true);
	CHECK_CUDA(cudaStreamSynchronize(streamA->get()));

	// Destroy streamA so it is unusable.
	// It will cause an error meessage on second attempt to destroy, but it does not affect the test.
	CHECK_CUDA(cudaStreamDestroy(streamA->get()));
	EXPECT_THROW(array->resize(10, true, true), std::runtime_error);

	// After setting a new stream, resize should work.
	array->setStream(streamB);
	array->resize(10, true, true);
	CHECK_CUDA(cudaStreamSynchronize(streamB->get()));
}

TEST(Array, ChangeStreamViaManager)
{
	DeviceAsyncArrayManager arrayMgr;

	// If stream was not set, refuse to create array
	EXPECT_THROW(arrayMgr.create<int>(), std::logic_error);

	{
		// Set a stream that will expire
		auto stream = CudaStream::create();
		arrayMgr.setStream(stream);
	}
	// If previously set stream expired, refuse to create array
	EXPECT_THROW(arrayMgr.create<int>(), std::logic_error);

	// Pass CudaStream to ArrayManager, created array should ensure liveliness
	auto streamA = CudaStream::create();
	arrayMgr.setStream(streamA);

	// Create arrays through managers
	auto arrayA = arrayMgr.create<int>();
	auto arrayB = arrayMgr.create<float>();
	auto arrayC = arrayMgr.create<float>();

	// Calling setStream twice with the same stream should have no effect
	arrayMgr.setStream(streamA);

	// Do some operations in the stream
	arrayA->resize(5, true, true);
	arrayB->resize(5, true, true);
	arrayC->resize(5, true, true);

	// Remove one of the arrays, arrayMgr should not touch it afterwards
	arrayC.reset();

	// This will cause an error from CUDA, which will be logged (it's OK)
	CHECK_CUDA(cudaStreamDestroy(streamA->get()));

	// The following calls should fail due to previous destruction of streamA
	EXPECT_THROW(arrayA->resize(10, true, true), std::runtime_error);
	EXPECT_THROW(arrayB->resize(10, true, true), std::runtime_error);

	// Provide a new stream for arrays
	arrayMgr.setStream(CudaStream::create());

	// Now arrays should work well.
	arrayA->resize(10, true, true);
	arrayB->resize(10, true, true);
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
