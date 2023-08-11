#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <memory/InvalidArrayCast.hpp>
#include <memory/HostPageableArray.hpp>
#include <memory/HostPinnedArray.hpp>
#include <memory/DeviceAsyncArray.hpp>
#include <memory/DeviceSyncArray.hpp>

using namespace ::testing;

/*
 * TEST PURPOSE:
 * Check that *Array type casting works as expected:
 * - Throws on bad casts: mismatched data type, mismatched subclass (memory kind)
 * - Works for correct casts: allows to convert from a concrete type to IAnyArray and back.
 */

template<typename T, typename U, template<typename> typename V, template<typename> typename  W>
struct ArrayTypes
{
	using Data = T;

	using WrongData = U;

	template<typename TT>
	using Subclass = V<TT>;

	template<typename TT>
	using WrongSubclass = W<TT>;
};

using CastingCombinations = ::testing::Types<
		ArrayTypes<int, float, DeviceAsyncArray, DeviceSyncArray>,
		ArrayTypes<int, float, DeviceAsyncArray, HostPageableArray>,
		ArrayTypes<int, float, DeviceAsyncArray, HostPinnedArray>,

		ArrayTypes<int, float, DeviceSyncArray, DeviceAsyncArray>,
		ArrayTypes<int, float, DeviceSyncArray, HostPageableArray>,
		ArrayTypes<int, float, DeviceSyncArray, HostPinnedArray>,

		ArrayTypes<int, float, HostPageableArray, DeviceAsyncArray>,
		ArrayTypes<int, float, HostPageableArray, DeviceSyncArray>,
		ArrayTypes<int, float, HostPageableArray, HostPinnedArray>,

		ArrayTypes<int, float, HostPinnedArray, DeviceAsyncArray>,
		ArrayTypes<int, float, HostPinnedArray, DeviceSyncArray>,
		ArrayTypes<int, float, HostPinnedArray, HostPageableArray>
>;

template <typename T>
class ArrayTyping : public ::testing::Test {
protected:
	using Data = typename T::Data;
	using WrongData = typename T::WrongData;

	template<typename TT>
	using Subclass = typename T::template Subclass<TT>;

	template<typename TT>
	using WrongSubclass = typename T::template WrongSubclass<TT>;
};
TYPED_TEST_SUITE(ArrayTyping, CastingCombinations);

// Array creation helper, because DeviceAsyncArray does not have ::create() with no arguments like other kinds.
template<typename T, template<typename> typename Subclass>
Subclass<T>::Ptr createArray()
{
	if constexpr (std::is_same<Subclass<T>, DeviceAsyncArray<T>>::value) {
		return DeviceAsyncArray<T>::createStandalone(CudaStream::getNullStream());
	}
	else {
		return Subclass<T>::create();
	}
};

TYPED_TEST(ArrayTyping, Typing) {
	// Here, you can use Type, WrongType, RealSubclass, and WrongSubclass as if they were type aliases
	// For example:
	using SubclassType = typename TypeParam::template Subclass<typename TypeParam::Data>;
	typename SubclassType::Ptr arrayOriginal = createArray<typename TypeParam::Data, TypeParam::template Subclass>();

	IAnyArray::Ptr arrayAny = arrayOriginal->asAnyArray();
	arrayAny->resize(1, true, false);

	{
		// Attempt casting to a wrong Array data type, should throw
		// Lambda because comma in explicit template parameter list fools GTEST macros
		auto cast = [&]() { arrayOriginal->template asTypedArray<typename TypeParam::WrongData, TypeParam::template Subclass>(); };
		EXPECT_THROW(cast(), InvalidArrayCast);
	}

	{
		// Attempt casting to a wrong Array subclass, should throw
		// Lambda because comma in explicit template parameter list fools GTEST macros
		auto cast = [&]() { arrayOriginal->template asTypedArray<typename TypeParam::Data, TypeParam::template WrongSubclass>(); };
		EXPECT_THROW(cast(), InvalidArrayCast);
	}
	// Continue your tests as usual

	typename SubclassType::ConstPtr arrayOfTypeRecovered = arrayAny->asTypedArray<typename TypeParam::Data, TypeParam::template Subclass>();
	EXPECT_THAT(arrayOfTypeRecovered, NotNull());
	EXPECT_THAT(arrayOfTypeRecovered, Eq(arrayOriginal));
	EXPECT_EQ(arrayOfTypeRecovered->getCount(), 1);
	EXPECT_EQ(arrayOfTypeRecovered->getSizeOf(), sizeof(typename TypeParam::Data));
}
