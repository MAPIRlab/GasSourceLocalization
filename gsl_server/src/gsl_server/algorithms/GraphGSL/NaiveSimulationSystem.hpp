#pragma once

#include "Simulations.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"

namespace GSL::Graph_internal
{
    class NaiveSimulationSystem
    {
    public:
        NaiveSimulationSystem(Options& options);
        SimWithResult SimulateSourceFromPoint(const std::shared_ptr<PlaceNode> entireMap, const Vector2& sourcePoint);
        CompleteMap AsCompleteMap(const std::shared_ptr<PlaceNode> entireMap, SimWithResult result);

        Options& options;

    private:
        std::map<std::shared_ptr<RoomNode>, std::optional<SimulationBlurMask>> blurMasks;
    };
} // namespace GSL::Graph_internal