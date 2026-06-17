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
    struct SingleScale
    {
        double scale;
        float residual;
    };
    SingleScale FitSingleScale(const std::vector<float>& simulated,
                               const std::vector<float>& observed,
                               const std::vector<float>& uncertainty);

    struct MultipleScales
    {
        std::vector<double> scales;
        float residual;
    };
    MultipleScales FitDoorwayScales(const std::vector<std::vector<float>>& simulated,
                           const std::vector<float>& observed,
                           const std::vector<float>& uncertainty);
}; // namespace GSL::NACCeres
