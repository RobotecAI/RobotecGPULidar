#pragma once

#include <math.h>
#include <vector>

std::pair<float, float> mean_and_stdev(std::vector<float> data);

template <typename T>
std::vector<float> computeDistances(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = sqrt(data[i].value[0] * data[i].value[0] + data[i].value[1] * data[i].value[1] + data[i].value[2] * data[i].value[2]);
    }
    return return_data;
}

template <typename T>
std::vector<float> computeAngles(const T* data, int size)
{
    std::vector<float> return_data(size);
    for (int i = 0; i < size; i++) {
        return_data[i] = std::atan2(data[i].value[2], data[i].value[0]);
    }
    return return_data;
}
