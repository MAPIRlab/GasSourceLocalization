#pragma once
#include "Simulation.hpp"
#include "gsl_server/algorithms/Common/VisibilityMap.hpp"
#include <opencv2/core/mat.hpp>

namespace GSL
{
    struct Filament
    {
        Vector2 position;
        uint age = 0;
        int mostRecentOutlet = -1;
    };

    struct SimulationBlurMask
    {
        float sigma = 0.0;
        cv::Mat mask;
    };

    struct FilamentOutletInfo
    {
        std::vector<size_t> exitsPerOutlet;
        std::vector<size_t> lastUpdateTime;
    };

    struct FilamentSimulation : public Simulation
    {
        FilamentSimulation(const SimulationSource& source,
                           const Grid2D<Vector2>& wind,
                           std::optional<SimulationOutlets> outlets = std::nullopt)
            : Simulation(source, wind, outlets) {}
        enum class Type
        {
            HitFrequency = 0,
            Cummulative = 1
        };
        bool warmup = false;
        float warmupAcceleration = 2.0;
        size_t timesteps = 200;
        float deltaTime = 0.1;
        float noiseSTDev = 0.1;
        float numFilamentsSecond = 10;

        size_t minWarmupIterations = 100;
        size_t maxWarmupIterations = 500;

        std::optional<std::reference_wrapper<const VisibilityMap>> visibilityMap;

        void Run(std::vector<float>& hitMap, Type type = FilamentSimulation::Type::HitFrequency);

        void makeSimulationImage();
        static void displayImage(const Grid2D<float>& hitMap, const std::string& imageName = "simResult", float raisePower = 1);
        static void blurHitMap(std::vector<float>& hitMap, float blurSigma, Grid2D<Occupancy> occupancy, std::optional<SimulationBlurMask>& blurredMask);

        size_t totalEmittedFilaments = 0; // to be read after the simulation ends

        std::optional<FilamentOutletInfo> filamentOutlets;

    private:
        bool moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const;
        bool moveAlongPath(Vector2& currentPosition, const Vector2Int& indexOrigin, const Vector2& end) const;

        template <typename UpdateFunc>
        bool filamentIsOutside(Filament& filament, Vector2 oldPos, size_t currentTimestep, UpdateFunc updateFunc);

        template <typename UpdateFunc>
        void _Run(std::vector<float>& hitMap, UpdateFunc updateFunc, Type type);

        size_t FilamentsToEmit(float deltaT);
        float rawMaxValue;
        float emissionCounter = 0;
    };
} // namespace GSL
