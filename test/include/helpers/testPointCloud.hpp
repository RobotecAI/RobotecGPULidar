#pragma once

#include <random>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <helpers/commonHelpers.hpp>

static std::random_device randomDevice;
static unsigned randomSeed = randomDevice();
static std::mt19937 randomGenerator{randomSeed};

static const std::vector<rgl_field_t> allNotDummyFields = {XYZ_VEC3_F32,   INTENSITY_F32, IS_HIT_I32,  RAY_IDX_U32,
                                                           ENTITY_ID_I32,  RING_ID_U16,   AZIMUTH_F32, DISTANCE_F32,
                                                           RETURN_TYPE_U8, TIME_STAMP_F64};

static const std::vector<rgl_field_t> availablePaddings = {PADDING_8, PADDING_16, PADDING_32};

/**
 * @brief Generates a vector of values of the specified RGL PointCloud Field.
 *
 * @tparam FieldType The RGL Field type of data to generate.
 * @param count The number of elements to generate.
 * @param generator A user-specified function that defines how to generate each value based on its index.
 * @return A vector of generated values.
 */
template<typename FieldType>
static std::vector<FieldType> generateFieldValues(std::size_t count, std::function<FieldType(int)> generator)
{
	std::vector<FieldType> values;
	values.reserve(count);
	for (int i = 0; i < count; i++) {
		values.emplace_back(generator(i));
	}
	return values;
}

/**
 * Collection of generator functions for various RGL PointCloud Fields.
 * These can be passed as the 'generator' argument to the `generate` function.
*/
static std::function<Field<XYZ_VEC3_F32>::type(int)> genCoord = [](int i) {
	return Vec3f(static_cast<float>(i) / (static_cast<float>(i) + 1), static_cast<float>(i) / (static_cast<float>(i) + 2),
	             static_cast<float>(i) / (static_cast<float>(i) + 3));
};
static std::function<Field<INTENSITY_F32>::type(int)> genIntensity = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<AZIMUTH_F32>::type(int)> genAzimuth = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<DISTANCE_F32>::type(int)> genDistance = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<TIME_STAMP_F64>::type(int)> genTimeStamp = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<RAY_IDX_U32>::type(int)> genRayIdx = [](int i) { return i; };
static std::function<Field<ENTITY_ID_I32>::type(int)> genEntityId = [](int i) { return i; };
static std::function<Field<RETURN_TYPE_U8>::type(int)> genReturnType = [](int i) { return i % 3; };
static std::function<Field<RING_ID_U16>::type(int)> genRingId = [](int i) { return i; };

static std::function<Field<IS_HIT_I32>::type(int)> genHalfHit = [](int i) { return i % 2; };
static std::function<Field<IS_HIT_I32>::type(int)> genAllNonHit = [](int i) { return 0; };
static std::function<Field<IS_HIT_I32>::type(int)> genAllHit = [](int i) { return 1; };
static std::function<Field<IS_HIT_I32>::type(int)> genRandHit = [](int i) {
	return std::uniform_int_distribution<int>(0, 1)(randomGenerator);
};

/**
 * @brief A utility for managing point clouds using RGL fields.
 *
 * @details
 * The `TestPointCloud` class facilitates the construction, manipulation, and extraction of point clouds based on RGL fields.
 * Each point's data is a continuous byte block, structured by the cloud's defined fields. Users should be mindful of these fields,
 * especially when invoking operations that rely on a specific field, like transformations depending on XYZ_VEC3_F32.
 * Additionally, this class provides functionality to instantiate a point cloud from a node and to create a node from the point cloud.
 * By default, all fields are initialized to dummy values, and all paddings are initialized to 0.
 *
 * @example
 * std::vector<rgl_field_t> fields = { ... }; // Define your fields here
 * std::vector<XYZ_VEC3_F32> xyzValues = ...;     // Define your XYZ values here
 * TestPointCloud pointCloud(fields, 100);   // Create a point cloud with 100 points
 * pointCloud.setFieldValues<XYZ_VEC3_F32>(xyzValues); // Set the XYZ values for each point
 * Mat3x4f transformMatrix = ...; // Define your transformation matrix
 * pointCloud.transform(transformMatrix);    // Apply transformation
 * TestPointCloud fromNode = TestPointCloud::createFromNode(transformNode, fields); // Create a point cloud from a node
 * EXPECT_EQ(pointCloud, fromNode); // Compare point clouds
 */

class TestPointCloud
{
public:
	explicit TestPointCloud(const std::vector<rgl_field_t>& declaredFields, std::size_t pointCount) : fields(declaredFields)
	{
		resize(pointCount);
		initializeFieldValueGenerators(pointCount);

		std::size_t offset = 0;
		for (auto& field : fields) {
			offsets.emplace_back(offset);
			offset += getFieldSize(field);
		}

		// Initialize all fields to dummy values; paddings are initialized to 0
		for (const auto& field : fields) {
			if (isDummy(field)) {
				continue;
			}
			auto gen = fieldGenerators.find(field);
			if (gen != fieldGenerators.end()) {
				gen->second(pointCount);
			}
		}
	}

	static TestPointCloud createFromNode(rgl_node_t outputNode, const std::vector<rgl_field_t>& fields)
	{
		int32_t outCount;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(outputNode, fields.at(0), &outCount, nullptr));

		TestPointCloud pointCloud = TestPointCloud(fields, outCount);

		if (outCount == 0) {
			return pointCloud;
		}

		for (int i = 0; i < fields.size(); i++) {
			int32_t currentCount, currentSize;
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(outputNode, fields.at(i), &currentCount, &currentSize));
			EXPECT_EQ(currentSize, getFieldSize(fields.at(i)));
			EXPECT_EQ(outCount, currentCount);

			if (isDummy(fields.at(i))) {
				continue;
			}

			std::vector<char> dataFromNode;
			dataFromNode.resize(currentCount * currentSize);
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(outputNode, fields.at(i), dataFromNode.data()));
			for (int j = 0; j < currentCount; j++) {
				std::move(dataFromNode.begin() + j * currentSize, dataFromNode.begin() + (j + 1) * currentSize,
				          pointCloud.data.data() + j * pointCloud.getPointByteSize() + pointCloud.offsets.at(i));
			}
		}

		return pointCloud;
	}

	static TestPointCloud createFromFormatNode(rgl_node_t formatNode, const std::vector<rgl_field_t>& fields)
	{
		int32_t outCount, outSize;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(formatNode, RGL_FIELD_DYNAMIC_FORMAT, &outCount, &outSize));

		std::size_t expectedSize = 0;
		for (const auto& field : fields) {
			expectedSize += getFieldSize(field);
		}
		if (outSize != expectedSize) {
			throw std::invalid_argument("TestPointCloud::createFromFormatNode: formatNode does not match the expected size");
		}

		TestPointCloud pointCloud = TestPointCloud(fields, outCount);

		if (outCount == 0) {
			return pointCloud;
		}

		EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(formatNode, RGL_FIELD_DYNAMIC_FORMAT, pointCloud.data.data()));

		// If paddings are present, make their values 0, it will make the comparison easier
		for (int i = 0; i < fields.size(); ++i) {
			if (isDummy(fields.at(i))) {
				for (int j = 0; j < outCount; ++j) {
					// Pading bytes as 0
					std::fill(pointCloud.data.begin() + j * pointCloud.getPointByteSize() + pointCloud.offsets.at(i),
					          pointCloud.data.begin() + j * pointCloud.getPointByteSize() + pointCloud.offsets.at(i) +
					              getFieldSize(fields.at(i)),
					          0);
				}
			}
		}

		return pointCloud;
	}

	/**
	 * TODO(nebraszka): Consider adding the ability to iterate over points in TestPointCloud and delete selected ones
	 *         instead of copying the data to a new vector. This would be more flexible, cleaner, and more efficient.
	 */
	void removeNonHitPoints()
	{
		std::vector<char> newData;
		newData.reserve(data.size());

		for (int i = 0; i < fields.size(); ++i) {
			if (fields.at(i) == IS_HIT_I32) {
				for (int j = 0; j < getPointCount(); ++j) {
					if (*reinterpret_cast<int*>(data.data() + j * getPointByteSize() + offsets.at(i)) == 0) {
						continue;
					}
					std::move(data.begin() + j * getPointByteSize(), data.begin() + (j + 1) * getPointByteSize(),
					          std::back_inserter(newData));
				}
				break;
			}
		}
		data = newData;
	}

	std::size_t getPointCount() const { return data.size() / getPointByteSize(); }

	void transform(const Mat3x4f& transform)
	{
		// TODO: Once XYZ is separated from distance+vector spaces, this check will need to be updated.
		if (std::find(fields.begin(), fields.end(), XYZ_VEC3_F32) == fields.end()) {
			throw std::invalid_argument("TestPointCloud::transform: TestPointCloud does not contain XYZ_VEC3_F32 field");
		} else {
			std::vector<Field<XYZ_VEC3_F32>::type> points = getFieldValues<XYZ_VEC3_F32>();
			for (auto& point : points) {
				point = transform * point;
			}
			setFieldValues<XYZ_VEC3_F32>(points);
		}
	}

	rgl_node_t createUsePointsNode() const
	{
		rgl_node_t node = nullptr;
		EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&node, data.data(), getPointCount(), fields.data(), fields.size()));
		EXPECT_THAT(node, testing::NotNull());

		return node;
	}

	bool operator==(const TestPointCloud& other) const
	{
		if (getPointCount() != other.getPointCount()) {
			return false;
		}
		if (fields != other.fields) {
			return false;
		}
		if (data != other.data) {
			return false;
		}
		return true;
	}

	template<rgl_field_t T>
	void setFieldValues(std::vector<typename Field<T>::type> fieldValues)
	{
		if (getPointCount() != fieldValues.size()) {
			throw std::invalid_argument("TestPointCloud::setFieldValues: pointCount does not match");
		}

		// There may be multiple fields of the same type, so we need to find all of them
		std::vector<std::size_t> fieldIndices;
		for (int i = 0; i < fields.size(); i++) {
			if (fields.at(i) == T) {
				fieldIndices.emplace_back(i);
			}
		}

		for (std::size_t fieldIndex : fieldIndices) {
			std::size_t offset = offsets.at(fieldIndex);
			for (std::size_t i = 0; i < getPointCount(); i++) {
				std::move(reinterpret_cast<char*>(fieldValues.data() + i), reinterpret_cast<char*>(fieldValues.data() + i + 1),
				          data.data() + i * getPointByteSize() + offset);
			}
		}
	}

	template<rgl_field_t T>
	std::vector<typename Field<T>::type> getFieldValues()
	{
		// There may be multiple fields of the same type, we need to find just one
		std::size_t fieldIndex = 0;

		for (std::size_t i = 0; i < fields.size(); i++) {
			if (fields.at(i) == T) {
				fieldIndex = i;
				break;
			}
		}

		std::vector<typename Field<T>::type> fieldValues;
		fieldValues.reserve(getPointCount());

		std::size_t offset = offsets.at(fieldIndex);

		for (std::size_t i = 0; i < getPointCount(); i++) {
			fieldValues.emplace_back(
			    *reinterpret_cast<typename Field<T>::type*>(data.data() + i * getPointByteSize() + offset));
		}

		return fieldValues;
	}

	std::size_t getPointByteSize() const
	{
		std::size_t pointByteSize = 0;
		for (const auto& field : fields) {
			pointByteSize += getFieldSize(field);
		}
		return pointByteSize;
	}

	char* getData() { return data.data(); }

	std::vector<rgl_field_t> getFields() { return fields; }

	~TestPointCloud() = default;

private:
	void resize(std::size_t newCount) { data.resize(newCount * getPointByteSize()); }

	std::vector<char> data;

	std::vector<rgl_field_t> fields;
	std::vector<int> offsets;

	std::map<rgl_field_t, std::function<void(std::size_t)>> fieldGenerators;

	void initializeFieldValueGenerators(std::size_t pointCount)
	{
		// clang-format off
		fieldGenerators = {
		    {  XYZ_VEC3_F32, [&](std::size_t count) { setFieldValues<XYZ_VEC3_F32>(generateFieldValues(count, genCoord)); }},
		    {    IS_HIT_I32, [&](std::size_t count) { setFieldValues<IS_HIT_I32>(generateFieldValues(count, genRandHit)); }},
		    {   RAY_IDX_U32, [&](std::size_t count) { setFieldValues<RAY_IDX_U32>(generateFieldValues(count, genRayIdx)); }},
		    { ENTITY_ID_I32, [&](std::size_t count) { setFieldValues<ENTITY_ID_I32>(generateFieldValues(count, genEntityId)); }},
		    { INTENSITY_F32, [&](std::size_t count) { setFieldValues<INTENSITY_F32>(generateFieldValues(count, genIntensity)); }},
		    {   RING_ID_U16, [&](std::size_t count) { setFieldValues<RING_ID_U16>(generateFieldValues(count, genRingId)); }},
		    {   AZIMUTH_F32, [&](std::size_t count) { setFieldValues<AZIMUTH_F32>(generateFieldValues(count, genAzimuth)); }},
		    {  DISTANCE_F32, [&](std::size_t count) { setFieldValues<DISTANCE_F32>(generateFieldValues(count, genDistance)); }},
		    {RETURN_TYPE_U8, [&](std::size_t count) { setFieldValues<RETURN_TYPE_U8>(generateFieldValues(count, genReturnType)); }},
		    {TIME_STAMP_F64, [&](std::size_t count) { setFieldValues<TIME_STAMP_F64>(generateFieldValues(count, genTimeStamp)); }}
        };
		// clang-format on
	}
};

/**
 * @brief Function to generate a vector of fields; returns a vector of all not dummy fields with the specified number of paddings.
 * Paddings types and positions are chosen randomly.
 */

static std::vector<rgl_field_t> generateRandomStaticFieldsVector(std::size_t numberOfPaddings)
{
	std::vector<rgl_field_t> fields;
	fields.reserve(allNotDummyFields.size() + numberOfPaddings);

	std::sample(allNotDummyFields.begin(), allNotDummyFields.end(), std::back_inserter(fields), allNotDummyFields.size(),
	            randomGenerator);

	// Insert paddings at random positions
	for (int i = 0; i < numberOfPaddings; ++i) {
		std::uniform_int_distribution<> posDis(0, fields.size());
		int position = posDis(randomGenerator);
		// Random padding from availablePaddings
		fields.insert(fields.begin() + position,
		              availablePaddings[std::uniform_int_distribution<>(0, availablePaddings.size() - 1)(randomGenerator)]);
	}

	return fields;
}

/**
 * Functions for validating Field values.
 */

template<typename FieldType>
static void checkIfNearEqual(const std::vector<FieldType>& expected, const std::vector<FieldType>& actual,
                             const float epsilon = 1e-5)
{
	ASSERT_EQ(expected.size(), actual.size());
	for (int i = 0; i < expected.size(); ++i) {
		EXPECT_NEAR(expected[i], actual[i], epsilon);
	}
}

static void checkIfNearEqual(const std::vector<Field<XYZ_VEC3_F32>::type>& expected,
                             const std::vector<Field<XYZ_VEC3_F32>::type>& actual, const float epsilon = 1e-5)
{
	ASSERT_EQ(expected.size(), actual.size());
	for (int i = 0; i < expected.size(); ++i) {
		EXPECT_NEAR(expected[i][0], actual[i][0], epsilon);
		EXPECT_NEAR(expected[i][1], actual[i][1], epsilon);
		EXPECT_NEAR(expected[i][2], actual[i][2], epsilon);
	}
}
