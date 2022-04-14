#include "statistical_utils.h"

#include <numeric>

std::pair<float, float> mean_and_stdev(std::vector<float> v) {
    float sum = std::accumulate(v.begin(), v.end(), 0.0);
    float mean = sum / v.size();

    std::vector<float> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<float>(), mean));
    float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    float stdev = std::sqrt(sq_sum / v.size());

    return {mean, stdev};
}
