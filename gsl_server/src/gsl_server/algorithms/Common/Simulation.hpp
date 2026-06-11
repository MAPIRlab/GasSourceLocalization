#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/VisibilityMap.hpp"
#include "gsl_server/core/Vectors.hpp"
#include <opencv2/core/mat.hpp>
#include <optional>

namespace GSL
{
    struct Filament
    {
        Vector2 position;
        int mostRecentOutlet = -1;
    };

    struct SimulationSource
    {
        enum Mode
        {
            AABB,
            Point
        };

        const Mode mode;
        const std::optional<AABB2D> aabb;
        const Vector2 point;
        float numFilamentsSecond = 10;

        SimulationSource(const Vector2& _point)
            : mode(Mode::Point), aabb(std::nullopt), point(_point)
        {}
        SimulationSource(const AABB2D& _aabb)
            : mode(Mode::AABB), aabb(_aabb), point(0, 0)
        {}

        Vector2 getPoint() const;
        size_t FilamentsToEmit(float deltaT);

    private:
        float emissionCounter = 0;
    };

    struct SimulationOutlets
    {
        Grid2D<int> mask;
        size_t totalExitCount = 0;
        std::vector<size_t> exitsPerOutlet;
        std::vector<size_t> numCellsOutlet;
        std::vector<size_t> lastUpdateTime;
        std::vector<bool> enabled;
    };

    struct SimulationBlurMask
    {
        float sigma = 0.0;
        cv::Mat mask;
    };

    struct Simulation
    {
        enum class Type
        {
            HitFrequency = 0,
            Cummulative = 1
        };
        SimulationSource source;
        bool warmup = false;
        float warmupAcceleration = 2.0;
        size_t timesteps = 200;
        float deltaTime = 0.1;
        float noiseSTDev = 0.1;

        size_t minWarmupIterations = 100;
        size_t maxWarmupIterations = 500;

        Grid2D<Vector2> wind;
        std::optional<std::reference_wrapper<const VisibilityMap>> visibilityMap;
        std::optional<SimulationOutlets> outlets;

        void Run(std::vector<float>& hitMap, Type type = Simulation::Type::HitFrequency);

        void makeSimulationImage();
        static void displayImage(const Grid2D<float>& hitMap, const std::string& imageName = "simResult", float raisePower = 1);
        static void blurHitMap(std::vector<float>& hitMap, float blurSigma, Grid2D<Occupancy> occupancy, std::optional<SimulationBlurMask>& blurredMask);

        size_t totalEmittedFilaments = 0; // to be read after the simulation ends

    private:
        bool moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const;
        bool moveAlongPath(Vector2& currentPosition, const Vector2Int& indexOrigin, const Vector2& end) const;

        template <typename UpdateFunc>
        bool filamentIsOutside(Filament& filament, Vector2 oldPos, size_t currentTimestep, UpdateFunc updateFunc);

        template <typename UpdateFunc>
        void _Run(std::vector<float>& hitMap, UpdateFunc updateFunc, Type type);
    };
} // namespace GSL
