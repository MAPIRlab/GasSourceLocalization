#pragma once

#include "Simulations.hpp"
#include "gsl_server/algorithms/Common/Utils/Images.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"

namespace GSL::Graph_internal
{
    class NaiveSimulationSystem
    {
    public:
        NaiveSimulationSystem(FilamentSimOptions& options);
        SimWithResult SimulateSourceFromPoint(const std::shared_ptr<PlaceNode> entireMap, const Vector2& sourcePoint);
        CompleteMap AsCompleteMap(const std::shared_ptr<PlaceNode> entireMap, SimWithResult result);

        FilamentSimOptions& options;

    private:
        std::map<std::shared_ptr<RoomNode>, std::optional<Utils::Image::BlurMask>> blurMasks;
    };
} // namespace GSL::Graph_internal