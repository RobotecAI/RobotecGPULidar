#pragma once

#include <vector>
#include <numeric>
#include <cmath>

template<typename T>
std::pair<T, T> calcMeanAndStdev(std::vector<T> v) {
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	float mean = sum / v.size();

	std::vector<float> diff(v.size());
	std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
	float sqSum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	float stdev = std::sqrt(sqSum / v.size());

	return {mean, stdev};
}