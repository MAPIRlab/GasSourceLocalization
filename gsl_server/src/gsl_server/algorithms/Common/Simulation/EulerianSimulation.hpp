#pragma once
#include "Simulation.hpp"

namespace GSL
{
    struct EulerianSimulation : public Simulation
    {
        EulerianSimulation(const SimulationSource& source,
                           const Grid2D<Vector2>& wind,
                           std::optional<SimulationOutlets> outlets = std::nullopt)
            : Simulation(source, wind, outlets) {}

        void Run(std::vector<float>& gasMap, float lowerThr, std::string roomID);
        static void ClearAllCaches();
        static void ClearCacheRoom(std::string roomID);
    };
} // namespace GSL