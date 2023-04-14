#pragma once

#include <random>

#include <RGLFields.hpp>
#include <math/Mat3x4f.hpp>
#include <helpers/commonHelpers.hpp>

static std::random_device randomDevice;
static unsigned randomSeed = randomDevice();
static std::mt19937 randomGenerator{randomSeed};

/**
 * @brief A utility for managing point clouds using RGL fields.
 *
 * @details
 * The `TestPointCloud` class facilitates the construction, manipulation, and extraction of point clouds based on RGL fields.
 * Each point's data is a continuous byte block, structured by the cloud's defined fields. Users should be mindful of these fields,
 * especially when invoking operations that rely on a specific field, like transformations depending on XYZ_VEC3_F32.
 * Additionally, this class provides functionality to instantiate a point cloud from a node and to create a node from the point cloud.
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

		std::size_t offset = 0;
		for (const auto& field : fields) {
			offsets[field] = offset;
			offset += getFieldSize(field);
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

		for (auto& field : fields) {
			int32_t currentCount, currentSize;
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(outputNode, field, &currentCount, &currentSize));
			EXPECT_EQ(currentSize, getFieldSize(field));
			EXPECT_EQ(outCount, currentCount);

			if (isDummy(field)) {
				continue;
			}

			std::vector<char> dataFromNode;
			dataFromNode.resize(currentCount * currentSize);
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(outputNode, field, dataFromNode.data()));
			for (int j = 0; j < currentCount; j++) {
				std::move(dataFromNode.begin() + j * currentSize, dataFromNode.begin() + (j + 1) * currentSize,
				          pointCloud.data.data() + j * pointCloud.getPointByteSize() + pointCloud.offsets.at(field));
			}
		}

		return pointCloud;
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

		std::size_t offset = offsets.at(T);

		for (std::size_t i = 0; i < getPointCount(); i++) {
			std::move(reinterpret_cast<char*>(fieldValues.data() + i), reinterpret_cast<char*>(fieldValues.data() + i + 1),
			          data.data() + i * getPointByteSize() + offset);
		}
	}

	template<rgl_field_t T>
	std::vector<typename Field<T>::type> getFieldValues()
	{
		std::vector<typename Field<T>::type> fieldValues;
		fieldValues.reserve(getPointCount());

		std::size_t offset = offsets.at(T);

		for (std::size_t i = 0; i < getPointCount(); i++) {
			fieldValues.emplace_back(
			    *reinterpret_cast<typename Field<T>::type*>(data.data() + i * getPointByteSize() + offset));
		}

		return fieldValues;
	}

	char* getData() { return data.data(); }

	~TestPointCloud() = default;

private:
	void resize(std::size_t newCount) { data.resize(newCount * getPointByteSize()); }

	std::size_t getPointByteSize() const
	{
		std::size_t pointByteSize = 0;
		for (const auto& field : fields) {
			pointByteSize += getFieldSize(field);
		}
		return pointByteSize;
	}

	std::vector<char> data;
	std::vector<rgl_field_t> fields;
	std::map<rgl_field_t, std::size_t> offsets;
};

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
static std::function<Field<XYZ_VEC3_F32>::type(int)> genCoord = [](int i) { return Vec3f(i, i + 1, i + 2); };
static std::function<Field<RAY_IDX_U32>::type(int)> genRayIdx = [](int i) { return i; };
static std::function<Field<ENTITY_ID_I32>::type(int)> genEntityId = [](int i) { return i; };
static std::function<Field<INTENSITY_F32>::type(int)> genIntensity = [](int i) { return i * 1.1f; };
static std::function<Field<RING_ID_U16>::type(int)> genRingId = [](int i) { return i; };
static std::function<Field<AZIMUTH_F32>::type(int)> genAzimuth = [](int i) { return i * 1.1f; };
static std::function<Field<DISTANCE_F32>::type(int)> genDistance = [](int i) { return i * 1.1f; };
static std::function<Field<RETURN_TYPE_U8>::type(int)> genReturnType = [](int i) { return i % 3; };
static std::function<Field<TIME_STAMP_F64>::type(int)> genTimeStamp = [](int i) { return i * 1.1; };

static std::function<Field<IS_HIT_I32>::type(int)> genHalfHit = [](int i) { return i % 2; };
static std::function<Field<IS_HIT_I32>::type(int)> genAllNonHit = [](int i) { return 0; };
static std::function<Field<IS_HIT_I32>::type(int)> genAllHit = [](int i) { return 1; };
static std::function<Field<IS_HIT_I32>::type(int)> genRandHit = [](int i) {
	return std::uniform_int_distribution<int>(0, 1)(randomGenerator);
};

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

/**
* Other helper functions.
*/

static rgl_node_t simulateEmptyPointCloudOutputNode()
{
	// Create TestPointCloud with one non hit point
	int pointsCount = 1;
	TestPointCloud pointCloud({XYZ_F32, IS_HIT_I32}, pointsCount);
	pointCloud.setFieldValues<XYZ_F32>(generateFieldValues(pointsCount, genCoord));
	pointCloud.setFieldValues<IS_HIT_I32>(generateFieldValues(pointsCount, genAllNonHit));

	// Prepare graph
	rgl_node_t usePointsNode = pointCloud.createUsePointsNode();
	rgl_node_t compactNode = nullptr;
	EXPECT_RGL_SUCCESS(rgl_node_points_compact(&compactNode));
	EXPECT_THAT(compactNode, testing::NotNull());
	EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(usePointsNode, compactNode));

	return compactNode;
}