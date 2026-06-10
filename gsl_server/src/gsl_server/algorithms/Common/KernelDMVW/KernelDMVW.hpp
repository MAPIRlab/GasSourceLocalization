#pragma once

#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"

namespace GSL::KernelDMVW
{
    struct KernelCell
    {
        double omega = 0;      // intermediate step for the confidence value, goes from 0 to +infinity
        double confidence = 0; // 0-1
        Utils::RunningVariance meanAndVariance;

        float ExpectedConcentration(float defaultValue = 0.f) { return std::lerp(defaultValue, meanAndVariance.mean, confidence); }
    };

    class GasMap
    {
    public:
        struct Params
        {
            float kernelSigma = 0.3;               // controls the falloff of the weight as a function of distance
            float kernelStretchConstant = 0.5;     // how much the default sigma is modified by the wind
            float sigmaOmega = 2.0;                // Controls how much confidence you gain from one measurement
            float omegaConcentrationSpatial = 5.0; // how much the confidence spreads to other cells
        };
        GasMap(Grid2D<Occupancy> occupancyMap, const Params& parameters);
        void AddReading(float concentration, Vector2 wind, Vector2 position);
        const Grid2D<KernelCell> GetMap() { return Grid2D<KernelCell>(cells, occupancy, metadata); }

    private:
        std::vector<KernelCell> cells;
        Grid2DMetadata metadata;
        std::vector<Occupancy> occupancy;
        Params params;
    };
} // namespace GSL::KernelDMVW