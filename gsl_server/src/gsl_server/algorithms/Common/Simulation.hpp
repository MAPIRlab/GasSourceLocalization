#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/VisibilityMap.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <optional>

namespace GSL
{
    struct Filament
    {
        Vector2 position;
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

        SimulationSource(const Vector2& _point)
            : mode(Mode::Point), aabb(std::nullopt), point(_point)
        {}
        SimulationSource(const AABB2D& _aabb)
            : mode(Mode::AABB), aabb(_aabb), point(0, 0)
        {}

        Vector2 getPoint() const;
    };

    struct Outlets
    {
        Grid2D<int> mask;
        std::vector<size_t> exitsCount;
        std::vector<bool> enabled;
    };

    struct Simulation
    {
        SimulationSource source;
        bool warmup = false;
        size_t timesteps = 200;
        float deltaTime = 0.1;
        float noiseSTDev = 0.15;

        size_t minWarmupIterations = 100;
        size_t maxWarmupIterations = 500;

        Grid2D<Vector2> wind;
        std::optional<std::reference_wrapper<VisibilityMap>> visibilityMap;
        std::optional<Outlets> outlets;

        void Run(std::vector<float>& hitMap);
        void moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const;
        bool filamentIsOutside(const Filament& filament);
        bool moveAlongPath(Vector2& beginning, const Vector2& end) const;

        void makeSimulationImage();
        void displayImage(const std::vector<float>& hitMap, const std::string& imageName = "simResult") const;

        size_t totalEmittedFilaments = 0; // to be read after the simulation ends
    };
} // namespace GSL
