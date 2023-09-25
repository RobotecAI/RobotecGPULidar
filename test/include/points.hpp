#pragma once

#include <utils.hpp>

class PointCloud
{
public:
	explicit PointCloud(const std::vector<rgl_field_t>& declaredFields, size_t pointCount)
	{
		fields = declaredFields;
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
				          pointCloud.data.data() + j * pointCloud.getPointByteSize() + pointCloud.offsets.at(fields.at(i)));
			}
		}

		return pointCloud;
	}

	std::size_t getPointCount() const { return data.size() / getPointByteSize(); }

	void resize(std::size_t newCount) { data.resize(newCount * getPointByteSize()); }

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

	std::vector<char> data;
	std::vector<rgl_field_t> fields;
	std::map<rgl_field_t, std::size_t> offsets;
};

// ---------------------- Field values generators ----------------------

static std::vector<Field<XYZ_F32>::type> generateCoord(std::size_t count)
{
	std::vector<Field<XYZ_F32>::type> coords;
	coords.reserve(count);
	for (int i = 0; i < count; i++) {
		coords.push_back(Vec3f(i, i + 1, i + 2));
	}
	return coords;
}

enum class HitPointDensity
{
	HALF_HIT = 0,
	ALL_NON_HIT,
	ALL_HIT,
	RANDOM
};

template<HitPointDensity T>
struct Density
{};

#define DENSITY(NAME, FILLER)                                                                                                  \
	template<>                                                                                                                 \
	struct Density<NAME>                                                                                                       \
	{                                                                                                                          \
		std::function<int(int)> filler = FILLER;                                                                               \
	}

DENSITY(HitPointDensity::HALF_HIT, [](int i) { return i % 2; });
DENSITY(HitPointDensity::ALL_NON_HIT, [](int i) { return 0; });
DENSITY(HitPointDensity::ALL_HIT, [](int i) { return 1; });
DENSITY(HitPointDensity::RANDOM, [](int i) { return rand() % 2; }); // TODO change rand generator

static std::vector<Field<IS_HIT_I32>::type> generateIsHit(std::size_t count,
                                                          HitPointDensity density = HitPointDensity::HALF_HIT)
{
	std::vector<Field<IS_HIT_I32>::type> isHit;
	isHit.reserve(count);
	for (int i = 0; i < count; i++) {
		isHit.push_back(Density<HitPointDensity::HALF_HIT>().filler(i));
	}
	return isHit;
}

static std::vector<Field<RAY_IDX_U32>::type> generateRayIdx(std::size_t count)
{
	std::vector<Field<RAY_IDX_U32>::type> rayIdx;
	rayIdx.reserve(count);
	for (int i = 0; i < count; i++) {
		rayIdx.push_back(i);
	}
	return rayIdx;
}

static std::vector<Field<ENTITY_ID_I32>::type> generateEntityId(std::size_t count)
{
	std::vector<Field<ENTITY_ID_I32>::type> entityId;
	entityId.reserve(count);
	for (int i = 0; i < count; i++) {
		entityId.push_back(i);
	}
	return entityId;
}

static std::vector<Field<INTENSITY_F32>::type> generateIntensity(std::size_t count)
{
	std::vector<Field<INTENSITY_F32>::type> intensity;
	intensity.reserve(count);
	for (int i = 0; i < count; i++) {
		intensity.push_back(i * 1.1f);
	}
	return intensity;
}

static std::vector<Field<RING_ID_U16>::type> generateRingId(std::size_t count)
{
	std::vector<Field<RING_ID_U16>::type> ringId;
	ringId.reserve(count);
	for (int i = 0; i < count; i++) {
		ringId.push_back(i);
	}
	return ringId;
}

static std::vector<Field<AZIMUTH_F32>::type> generateAzimuth(std::size_t count)
{
	std::vector<Field<AZIMUTH_F32>::type> azimuth;
	azimuth.reserve(count);
	for (int i = 0; i < count; i++) {
		azimuth.push_back(i * 1.1f);
	}
	return azimuth;
}

static std::vector<Field<DISTANCE_F32>::type> generateDistance(std::size_t count)
{
	std::vector<Field<DISTANCE_F32>::type> distance;
	distance.reserve(count);
	for (int i = 0; i < count; i++) {
		distance.push_back(i * 1.1f);
	}
	return distance;
}

static std::vector<Field<RETURN_TYPE_U8>::type> generateReturnType(std::size_t count)
{
	std::vector<Field<RETURN_TYPE_U8>::type> returnType;
	returnType.reserve(count);
	for (int i = 0; i < count; i++) {
		returnType.push_back(i % 3);
	}
	return returnType;
}

static std::vector<Field<TIME_STAMP_F64>::type> generateTimeStamp(std::size_t count)
{
	std::vector<Field<TIME_STAMP_F64>::type> timeStamp;
	timeStamp.reserve(count);
	for (int i = 0; i < count; i++) {
		timeStamp.push_back(i * 1.1);
	}
	return timeStamp;
}
