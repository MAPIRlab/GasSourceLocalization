#pragma once
#include "Simulation.hpp"

namespace GSL
{
    struct EulerianSimulation : public Simulation
    {
        struct Options
        {
            float lowerThr = 1e-5;
            size_t maxIterations = 1e7;
            float maxWindSpeed = 0.2;
            float minRho = 0;
            float maxRho = 0.9;
            float uncertaintyBlurSigma = 3.f;
        } options;

        EulerianSimulation(const SimulationSource& source,
                           const Grid2D<Vector2>& wind,
                           const Options& opts,
                           std::optional<SimulationOutlets> outlets = std::nullopt)
            : Simulation(source, wind, outlets), options(opts) {}

        void Run(std::vector<float>& gasMap, std::string roomID, std::optional<std::reference_wrapper<std::vector<float>>> uncertainty);
        static void ClearAllCaches();
        static void ClearCacheRoom(std::string roomID);
    };
} // namespace GSL