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

	static PointCloud createFromNode(const rgl_node_t outputNode, const std::vector<rgl_field_t>& fields)
	{
		int32_t outCount;
		EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(outputNode, fields.at(0), &outCount, nullptr));

		PointCloud pointCloud = PointCloud(fields, outCount);

		for (int i = 0; i < fields.size(); i++) {
			int32_t currentCount, currentSize;
			EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(outputNode, fields.at(i), &currentCount, &currentSize));
			EXPECT_EQ(currentSize, getFieldSize(fields.at(i)));
			EXPECT_EQ(outCount, currentCount);

			std::vector<char> dataFromNode;
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
			throw std::runtime_error(
			    "PointCloud::transform: PointCloud does not contain XYZ_F32 field");
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
			throw std::runtime_error("PointCloud::setFieldValues: pointCount does not match");
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
