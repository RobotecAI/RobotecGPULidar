#pragma once

#include <utils.hpp>

class IPointCloud {
public:
    explicit IPointCloud(const std::vector<rgl_field_t>& declaredFields, size_t pointCount, bool randomFill = false)
    {
    }

    IPointCloud(const rgl_node_t outputNode, const std::vector<rgl_field_t>& fields)
    {
    }

    virtual std::size_t getPointCount() const = 0;

    virtual void resize(std::size_t newCount) = 0;
    virtual void transform(const Mat3x4f& transform) = 0;

    virtual rgl_node_t createUsePointsNode() const = 0;

    virtual bool operator==(const IPointCloud& other) const = 0;

    template <rgl_field_t T>
    void setFieldValues(std::vector<typename Field<T>::type> fieldValues)
    {
        // TODO
    }

    template <rgl_field_t T>
    void setRandomFieldValues()
    {
        // TODO
    }

    template <rgl_field_t T>
    std::vector<typename Field<T>::type> getFieldValues()
    {
        // TODO
        return {};
    }

    virtual ~IPointCloud() = default;
};
