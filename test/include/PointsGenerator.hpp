#include <random>
#include <utils.hpp>

static std::random_device randomDevice;
static auto randomSeed = randomDevice();
static std::mt19937 randomGenerator { randomSeed };

struct TestPoint {
    Field<XYZ_F32>::type xyz;
    Field<INTENSITY_F32>::type intensity;

    TestPoint() = default;
    TestPoint(Field<XYZ_F32>::type xyz, Field<INTENSITY_F32>::type intensity)
        : xyz(xyz), intensity(intensity) {}

    std::vector<rgl_field_t> getFields()
    {
        return { XYZ_F32, INTENSITY_F32 };
    }

    void transform(const Mat3x4f& transform)
    {
        xyz = transform * xyz;
    }

    bool operator==(const TestPoint& other) const
    {
        return xyz.x() == other.xyz.x() && xyz.y() == other.xyz.y() && xyz.z() == other.xyz.z() && intensity == other.intensity;
    }
};

struct TestPointIsHit : public TestPoint {
    Field<IS_HIT_I32>::type isHit;

    TestPointIsHit() = default;
    TestPointIsHit(Field<XYZ_F32>::type xyz, Field<INTENSITY_F32>::type intensity, Field<IS_HIT_I32>::type isHit)
        : TestPoint(xyz, intensity), isHit(isHit) {}

    std::vector<rgl_field_t> getFields()
    {
        auto baseFields = TestPoint::getFields();
        baseFields.push_back(IS_HIT_I32);
        return baseFields;
    }

    bool operator==(const TestPointIsHit& other) const
    {
        return TestPoint::operator==(other) && isHit == other.isHit;
    }
};

enum class HitPointDensity {
    HALF_HIT = 0,
    ALL_NON_HIT,
    ALL_HIT,
    RANDOM,
    NONE
};

template <typename PointType>
class RGLTestPointsGeneratorBase {
public:
    virtual void generateTestPoints(int pointsNumber)
    {
        inPoints.clear();
        for (int i = 0; i < pointsNumber; ++i) {
            auto currentPoint = createPoint(i);
            inPoints.emplace_back(currentPoint);
        }
    }

    std::vector<rgl_field_t> getPointFields() const
    {
        return PointType().getFields();
    }

    auto getPointsSize() const -> int { return inPoints.size(); }

    auto getPointsData() const -> const auto* { return inPoints.data(); }

    void transformPoints(const Mat3x4f& transform)
    {
        for (auto& point : inPoints) {
            point.transform(transform);
        }
    }

protected:
    std::vector<PointType> inPoints;

    virtual PointType createPoint(int i) = 0;
};

class RGLTestPointsGenerator : public RGLTestPointsGeneratorBase<TestPoint> {
public:
    RGLTestPointsGenerator() = default;

protected:
    TestPoint createPoint(int i) override
    {
        return TestPoint { { i, i + 1, i + 2 }, 100 };
    }
};

class RGLTestPointsIsHitGenerator : public RGLTestPointsGeneratorBase<TestPointIsHit> {
public:
    RGLTestPointsIsHitGenerator(HitPointDensity hitPointDensity)
        : hitPointDensity(hitPointDensity) { }

    int getRandomNonHitCount() const { return randomNonHitCount; }

    void generateTestPoints(int pointsNumber) override
    {
        randomNonHitCount = 0;
        RGLTestPointsGeneratorBase<TestPointIsHit>::generateTestPoints(pointsNumber);
    }

    std::vector<TestPointIsHit> separateHitPoints()
    {
        std::vector<TestPointIsHit> hitPoints;
        for (const auto& point : inPoints) {
            if (point.isHit == 1) {
                hitPoints.push_back(point);
            }
        }
        return hitPoints;
    }

protected:
    HitPointDensity hitPointDensity;
    int32_t randomNonHitCount = 0;

    TestPointIsHit createPoint(int i) override
    {
        return TestPointIsHit { { i, i + 1, i + 2 }, 100, determineIsHit(i) };
    }

    int32_t determineIsHit(int index)
    {
        switch (hitPointDensity) {
        case HitPointDensity::HALF_HIT:
            return index % 2;
        case HitPointDensity::ALL_NON_HIT:
            return 0;
        case HitPointDensity::ALL_HIT:
            return 1;
        case HitPointDensity::RANDOM: {
            int isHit = randomGenerator() % 2;
            if (!isHit) {
                randomNonHitCount++;
            }
            return isHit;
        }
        default:
            return 0;
        }
    }
};
