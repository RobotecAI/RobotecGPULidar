#pragma once

#include <rgl/api/experimental.h>

struct LidarNoiseParams {
    rgl_angular_noise_type_t angularNoiseType;
    float angularNoiseStDev;
    float angularNoiseMean;
    float distanceNoiseStDevBase;
    float distanceNoiseStDevRisePerMeter;
    float distanceNoiseMean;
};
