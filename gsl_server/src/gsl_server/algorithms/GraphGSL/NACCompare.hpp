#pragma once

#include "gsl_server/core/Vectors.hpp"
#include <numeric>
#include <vector>

namespace GSL::NAC
{
    // just calculate the least squares solution with the pseudoinverse
    // this does not consider at all the effect of outliers
    float LeastSquaresScale(const std::vector<float>& simulated,
                            const std::vector<float>& observed,
                            const std::vector<float>& uncertainty);

    // Evaluate the solution
    float Residual(const std::vector<float>& simulated,
                   const std::vector<float>& observed,
                   const std::vector<float>& confidence,
                   float scale);

    // TODO the simulated vector has been transposed in the gsl file! update this function if it's going to be used
    std::vector<float> LeastSquaresDoorwayCombination(const std::vector<float>& observed,
                                                      const std::vector<std::vector<float>>& simulated,
                                                      const std::vector<float>& uncertainty);

    float ResidualDoorways(const std::vector<float>& observed,
                           const std::vector<std::vector<float>>& simulated,
                           const std::vector<float>& confidence,
                           const std::vector<float>& weights);

} // namespace GSL::NAC

// ceres version
namespace GSL::NACCeres
{
    inline float defaultResidual = 1e-2;
    inline float alpha = 0.15;
    inline float p = 2.5;

    inline float DistanceWeight(Vector2 pos, Vector2 sourcePos)
    {
        float t = std::pow(vmath::length(sourcePos - pos) * alpha, p);
        float scale = std::lerp(1.0, 0.1, std::clamp(t, 0.0f, 1.0f));
        return scale;
    }

    struct Result
    {
        std::vector<float> residuals;
        float TotalResidual() { return std::accumulate(residuals.begin(), residuals.end(), 0.0f); }
    };

    struct SingleScale
    {
        double scale;
        Result result;
    };
    SingleScale FitSingleScale(const std::vector<float>& simulated,
                               const std::vector<float>& observed,
                               const std::vector<float>& uncertainty);

    struct MultipleScales
    {
        std::vector<double> scales;
        Result result;
    };
    MultipleScales FitDoorwayScales(const std::vector<std::vector<float>>& simulated,
                                    const std::vector<float>& observed,
                                    const std::vector<float>& uncertainty);
}; // namespace GSL::NACCeres
