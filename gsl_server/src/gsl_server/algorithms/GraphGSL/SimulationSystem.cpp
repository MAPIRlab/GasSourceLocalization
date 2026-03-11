#include "SimulationSystem.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"

namespace GSL::Graph_internal
{
    SimulationSystem::SimWithResult SimulationSystem::SimulateFromArc(const Arc& arc)
    {
        SimWithResult result;
        auto realNode = As<RealNode>(arc.from.lock());
        Grid2DMetadata nodeMetadata = realNode->GetOccupancy().metadata;
        AABB2D sourceAABB(arc.aabb.min,
                          arc.aabb.max);
        Vector2 maxCoords = nodeMetadata.indicesToCoordinates(nodeMetadata.dimensions, true);
        sourceAABB.min.x = std::clamp(sourceAABB.min.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.min.y = std::clamp(sourceAABB.min.y, nodeMetadata.origin.y, maxCoords.y);
        sourceAABB.max.x = std::clamp(sourceAABB.max.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.max.y = std::clamp(sourceAABB.max.y, nodeMetadata.origin.y, maxCoords.y);

        // configure the simulation
        result.hitMap = std::make_shared<std::vector<float>>(realNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(sourceAABB),
            .minWarmupIterations = 1000,
            .maxWarmupIterations = 2000,
            .wind = realNode->GetWindMap(),
            .outlets = Outlets{
                .mask = realNode->GetOutletsMask(),
                .exitsCount = std::vector<size_t>(realNode->arcs.size(), 0),
            },
        });

        result.simulation->outlets->exitsCount.resize(realNode->arcs.size(), 0);
        result.simulation->outlets->enabled.resize(realNode->arcs.size(), true);

        for (size_t i = 0; i < realNode->arcs.size(); i++)
            if (&realNode->arcs.at(i) == &arc) // TODO
                result.simulation->outlets->enabled.at(i) = false;

        result.simulation->Run(*result.hitMap);

        return result;
    }

} // namespace GSL::Graph_internal