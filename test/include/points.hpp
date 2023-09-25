#pragma once

#include <utils.hpp>
#include <random>

static std::random_device randomDevice;
static auto randomSeed = randomDevice();
static std::mt19937 randomGenerator{randomSeed};

class PointCloud
{
public:
	explicit PointCloud(const std::vector<rgl_field_t>& declaredFields, std::size_t pointCount) : fields(declaredFields)
	{
		resize(pointCount);

		std::size_t offset = 0;
		for (const auto& field : fields) {
			offsets[field] = offset;
			offset += getFieldSize(field);
		}
	}

	static PointCloud createFromNode(rgl_node_t outputNode, const std::vector<rgl_field_t>& fields)
	{
		int32_t outCount;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(outputNode, fields.at(0), &outCount, nullptr));

		PointCloud pointCloud = PointCloud(fields, outCount);

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
		if (std::find(fields.begin(), fields.end(), XYZ_F32) == fields.end()) {
			throw std::invalid_argument("PointCloud::transform: PointCloud does not contain XYZ_F32 field");
		} else {
			std::vector<Field<XYZ_F32>::type> points = getFieldValues<XYZ_F32>();
			for (auto& point : points) {
				point = transform * point;
			}
			setFieldValues<XYZ_F32>(points);
		}
	}

	rgl_node_t createUsePointsNode() const
	{
		rgl_node_t node = nullptr;
		EXPECT_RGL_SUCCESS(rgl_node_points_from_array(&node, data.data(), data.size(), fields.data(), fields.size()));
		return node;
	}

	bool operator==(const PointCloud& other) const
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
			throw std::invalid_argument("PointCloud::setFieldValues: pointCount does not match");
		}

		std::size_t offset = offsets.at(T);
		std::size_t fieldSize = getFieldSize(T);

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
		std::size_t fieldSize = getFieldSize(T);

		for (std::size_t i = 0; i < getPointCount(); i++) {
			fieldValues.push_back(*reinterpret_cast<typename Field<T>::type*>(data.data() + i * getPointByteSize() + offset));
		}

		return fieldValues;
	}

	~PointCloud() = default;

private:
	std::size_t getPointByteSize() const
	{
		std::size_t pointByteSize = 0;
		for (const auto& field : fields) {
			pointByteSize += getFieldSize(field);
		}
		return pointByteSize;
	}

	void resize(std::size_t newCount) { data.resize(newCount * getPointByteSize()); }

	std::vector<char> data;
	std::vector<rgl_field_t> fields;
	std::map<rgl_field_t, std::size_t> offsets;
};

/******************************** Field values generators ********************************/

template<typename FieldType>
static std::vector<FieldType> generate(std::size_t count, std::function<FieldType(int)> generator)
{
	std::vector<FieldType> values;
	values.reserve(count);
	for (int i = 0; i < count; i++) {
		values.push_back(generator(i));
	}
	return values;
}

std::function<Field<XYZ_F32>::type(int)> genCoord = [](int i) { return Vec3f(i, i + 1, i + 2); };
std::function<Field<RAY_IDX_U32>::type(int)> genRayIdx = [](int i) { return i; };
std::function<Field<ENTITY_ID_I32>::type(int)> genEntityId = [](int i) { return i; };
std::function<Field<INTENSITY_F32>::type(int)> genIntensity = [](int i) { return i * 1.1f; };
std::function<Field<RING_ID_U16>::type(int)> genRingId = [](int i) { return i; };
std::function<Field<AZIMUTH_F32>::type(int)> genAzimuth = [](int i) { return i * 1.1f; };
std::function<Field<DISTANCE_F32>::type(int)> genDistance = [](int i) { return i * 1.1f; };
std::function<Field<RETURN_TYPE_U8>::type(int)> genReturnType = [](int i) { return i % 3; };
std::function<Field<TIME_STAMP_F64>::type(int)> genTimeStamp = [](int i) { return i * 1.1; };

std::function<Field<IS_HIT_I32>::type(int)> genHalfHit = [](int i) { return i % 2; };
std::function<Field<IS_HIT_I32>::type(int)> genAllNonHit = [](int i) { return 0; };
std::function<Field<IS_HIT_I32>::type(int)> genAllHit = [](int i) { return 1; };
std::function<Field<IS_HIT_I32>::type(int)> genRandHit = [](int i) {
	return std::uniform_int_distribution<int>(0, 1)(randomGenerator);
};
