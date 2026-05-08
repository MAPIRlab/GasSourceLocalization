#pragma once

#include <vector>

// just calculate the least squares solution with the pseudoinverse
// this does not consider at all the effect of outliers
float LeastSquaresScale(const std::vector<float>& simulated, const std::vector<float>& observed);

// Evaluate the solution
float LossFunction(const std::vector<float>& a, const std::vector<float>& b, float scale);
