#pragma once

#include "gsl_server/algorithms/Common/Simulation.hpp"
#include <vector>

namespace GSL
{
    class Arc;
    class RealNode;
}

namespace GSL::Graph_internal
{
    class SimulationSystem
    {
    public:
        struct SimWithResult
        {
            std::shared_ptr<Simulation> simulation;
            std::shared_ptr<std::vector<float>> hitMap;
        };

    public:
        static SimWithResult SimulateFromPoint(const std::shared_ptr<RealNode> node, Vector2 point);
        static SimWithResult SimulateFromArc(const Arc& arc);
    };
} // namespace GSL::Graph_internal