#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include "gsl_server/core/Vectors.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace GSL
{

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

    struct SimulationOutlets
    {
        Grid2D<int> mask;
        std::vector<float> concentrationExitingDoorway;
        std::vector<size_t> numCellsOutlet;
        std::vector<bool> enabled;
    };
    
    struct Simulation
    {
        SimulationSource source;
        Grid2D<Vector2> wind;
        std::optional<SimulationOutlets> outlets;

        Simulation(const SimulationSource& source,
                   const Grid2D<Vector2>& wind,
                   std::optional<SimulationOutlets> outlets = std::nullopt)
            : source(source), wind(wind), outlets(outlets) {}

        virtual ~Simulation() = default; // we need something virtual so the compiler considers the type polymorphic and lets us do dynamic_cast

        static void displayImage(const Grid2D<float>& hitMap, const std::string& imageName = "simResult", float raisePower = 1);
    };
} // namespace GSL