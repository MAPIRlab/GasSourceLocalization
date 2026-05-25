#pragma once

#include <vector>

namespace GSL::NAC
{
    // just calculate the least squares solution with the pseudoinverse
    // this does not consider at all the effect of outliers
    float LeastSquaresScale(const std::vector<float>& simulated,
                            const std::vector<float>& observed,
                            const std::vector<float>& uncertainty);

    // Evaluate the solution
    float LossFunction(const std::vector<float>& simulated,
                       const std::vector<float>& observed,
                       const std::vector<float>& confidence,
                       float scale);
} // namespace GSL::NAC
