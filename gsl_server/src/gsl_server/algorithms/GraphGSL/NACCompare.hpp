#pragma once

#include <vector>

namespace GSL
{
    // just calculate the least squares solution with the pseudoinverse
    // this does not consider at all the effect of outliers
    float LeastSquaresScale(const std::vector<float>& simulated,
                            const std::vector<float>& observed,
                            const std::vector<float>& uncertainty);

    // Evaluate the solution
    float LossFunction(const std::vector<float>& a,
                       const std::vector<float>& b,
                       const std::vector<float>& uncertainty,
                       float scale);
} // namespace GSL
