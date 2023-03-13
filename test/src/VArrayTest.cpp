#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <VArray.hpp>
#include <VArrayProxy.hpp>

using namespace ::testing;

TEST(VArray, Smoke)
{
	VArrayProxy<int>::Ptr array = VArrayProxy<int>::create(1);
	int value = 4;

	{
		// Goal:
		// CPU: ||
		// GPU: |4|
		int* devicePtr = array->getWritePtr(MemLoc::Device);
		EXPECT_TRUE(devicePtr != nullptr);
		CHECK_CUDA(cudaMemcpy(devicePtr, &value, sizeof(int), cudaMemcpyDefault));
	}
	{
		// CPU: |4|
		// GPU: |4|
		const int* hostPtr = reinterpret_cast<const int*>(array->getReadPtr(MemLoc::Host));
		EXPECT_TRUE(hostPtr != nullptr);
		EXPECT_EQ(hostPtr[0], value);
	}
	{
		// CPU: |4|3|0|
		// GPU: |4|
		array->resize(3);
		EXPECT_EQ(array->getCount(), 3);
		(*array)[1] = 3;
		// Last element should have been zero-initialized
	}
	{
		// CPU: |4|3|0|
		// GPU: |4|3|0|
		int values[3];
		const int* devicePtr = array->getReadPtr(MemLoc::Device);
		CHECK_CUDA(cudaMemcpy(values, devicePtr, 3 * sizeof(int), cudaMemcpyDefault));
		EXPECT_EQ(values[0], 4);
		EXPECT_EQ(values[1], 3);
		EXPECT_EQ(values[2], 0);
	}
}

TEST(VArray, SmokeInsert)
{
	// Default location is Device
	VArrayProxy<int>::Ptr array = VArrayProxy<int>::create(1);

	{
		// Goal:
		// CPU: ||
		// GPU: |1|2|3|4|5|
		array->resize(3);
		int value = 1;
		array->insertData(&value, 1, 0);
		value = 2;
		array->insertData(&value, 1, 1);
		value = 3;
		array->insertData(&value, 1, 2);
		// Next inserts should resize array with preserveData
		value = 4;
		array->insertData(&value, 1, 3);
		value = 5;
		array->insertData(&value, 1, 4);
	}
	{
		// CPU: |1|2|3|4|5|
		// GPU: |1|2|3|4|5|
		const int* hostPtr = reinterpret_cast<const int*>(array->getReadPtr(MemLoc::Host));
		EXPECT_TRUE(hostPtr != nullptr);
		EXPECT_EQ(hostPtr[0], 1);
		EXPECT_EQ(hostPtr[1], 2);
		EXPECT_EQ(hostPtr[2], 3);
		EXPECT_EQ(hostPtr[3], 4);
		EXPECT_EQ(hostPtr[4], 5);
	}
}
