#include <RGLFields.hpp>
#include <ranges>
#include <random>
#include <numbers>

static std::random_device randomDevice;
static unsigned randomSeed = randomDevice();
static std::mt19937 randomGenerator{randomSeed};

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
	auto view = std::views::iota(0, static_cast<int>(count)) | std::views::transform(generator);
	return std::vector<FieldType>(view.begin(), view.end());
}

/**
* Collection of generator functions for various RGL TestPointCloud Fields.
* These can be passed as the 'generator' argument to the `generateFieldValues` function.
*/
static std::function<Field<XYZ_VEC3_F32>::type(int)> genCoord = [](int i) {
	return Vec3f(static_cast<float>(i) / (static_cast<float>(i) + 1), static_cast<float>(i) / (static_cast<float>(i) + 2),
	             static_cast<float>(i) / (static_cast<float>(i) + 3));
};
static std::function<Field<INTENSITY_F32>::type(int)> genIntensityF32 = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<INTENSITY_U8>::type(int)> genIntensityU8 = [](int i) {
	return i % std::numeric_limits<Field<INTENSITY_U8>::type>::max();
};
static std::function<Field<LASER_RETRO_F32>::type(int)> genLaserRetro = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<AZIMUTH_F32>::type(int)> genAzimuth = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<ELEVATION_F32>::type(int)> genElevation = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<DISTANCE_F32>::type(int)> genDistance = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<TIME_STAMP_F64>::type(int)> genTimeStampF64 = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<TIME_STAMP_U32>::type(int)> genTimeStampU32 = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<RAY_IDX_U32>::type(int)> genRayIdx = [](int i) { return i; };
static std::function<Field<ENTITY_ID_I32>::type(int)> genEntityId = [](int i) { return i; };
static std::function<Field<RETURN_TYPE_U8>::type(int)> genReturnType = [](int i) { return i % 3; };
static std::function<Field<RING_ID_U16>::type(int)> genRingId = [](int i) { return i; };
static std::function<Field<ABSOLUTE_VELOCITY_VEC3_F32>::type(int)> genAbsoluteVelocity = [](int i) {
	return Vec3f(static_cast<float>(i) / (static_cast<float>(i) + 1), static_cast<float>(i) / (static_cast<float>(i) + 2),
	             static_cast<float>(i) / (static_cast<float>(i) + 3));
};
static std::function<Field<RELATIVE_VELOCITY_VEC3_F32>::type(int)> genRelativeVelocity = [](int i) {
	return Vec3f(static_cast<float>(i) / (static_cast<float>(i) + 1), static_cast<float>(i) / (static_cast<float>(i) + 2),
	             static_cast<float>(i) / (static_cast<float>(i) + 3));
};
static std::function<Field<RADIAL_SPEED_F32>::type(int)> genRadialSpeed = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<POWER_F32>::type(int)> genPower = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<RCS_F32>::type(int)> genRcs = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<NOISE_F32>::type(int)> genNoise = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<SNR_F32>::type(int)> genSnr = [](int i) {
	return static_cast<float>(i) / (static_cast<float>(i + 1));
};
static std::function<Field<INCIDENT_ANGLE_F32>::type(int)> genIncidentAngle = [](int i) {
	return std::uniform_real_distribution<float>(0, std::numbers::pi / 2.0f)(randomGenerator);
};
static std::function<Field<NORMAL_VEC3_F32>::type(int)> genNormal = [](int i) {
	std::mt19937 gen(i);
	std::uniform_real_distribution<> dis(0, 2 * M_PI);
	double theta = dis(gen), phi = dis(gen);
	return Vec3f{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
};
static std::function<Field<RAY_POSE_MAT3x4_F32>::type(int)> genRayPose = [](int i) {
	std::uniform_real_distribution<float> tr(-1, 1);
	std::uniform_real_distribution<float> rotDeg(-2 * std::numbers::pi, 2 * std::numbers::pi);
	return Mat3x4f::TRS(Vec3f(tr(randomGenerator), tr(randomGenerator), tr(randomGenerator)),
	                    Vec3f(rotDeg(randomGenerator), rotDeg(randomGenerator), rotDeg(randomGenerator)));
};

static std::function<Field<IS_HIT_I32>::type(int)> genHalfHit = [](int i) { return i % 2; };
static std::function<Field<IS_HIT_I32>::type(int)> genAllNonHit = [](int i) { return 0; };
static std::function<Field<IS_HIT_I32>::type(int)> genAllHit = [](int i) { return 1; };
static std::function<Field<IS_HIT_I32>::type(int)> genRandHit = [](int i) {
	return std::uniform_int_distribution<int>(0, 1)(randomGenerator);
};
static std::function<Field<IS_GROUND_I32>::type(int)> genAllGround = [](int i) { return 0; };
static std::function<Field<IS_GROUND_I32>::type(int)> genAllNonGround = [](int i) { return 1; };
static std::function<Field<IS_GROUND_I32>::type(int)> genRandGround = [](int i) {
	return std::uniform_int_distribution<int>(0, 1)(randomGenerator);
};