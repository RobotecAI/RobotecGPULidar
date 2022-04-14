#pragma once

enum AngularNoiseType { RAY_BASED = 0, HITPOINT_BASED = 1};

struct LidarNoiseParams {
    AngularNoiseType angularNoiseType;
    float angularNoiseStDev;
    float angularNoiseMean;
    float distanceNoiseStDevBase;
    float distanceNoiseStDevRisePerMeter;
    float distanceNoiseMean;
};
