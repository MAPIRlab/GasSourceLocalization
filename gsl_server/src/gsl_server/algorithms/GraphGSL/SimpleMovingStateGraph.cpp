#include "GraphGSL.hpp"
#include "SimpleMovingStateGraph.hpp"

namespace GSL
{
    SimpleMovingStateGraph::SimpleMovingStateGraph(Algorithm* alg) : ManualNavigationState(alg), gsl(dynamic_cast<GraphGSL*>(alg))
    {
        for (auto node : gsl->graph.nodes)
            if (auto roomNode = As<RoomNode>(node))
            {
                for (auto it = roomNode->GetOutletsMask().begin(); it != roomNode->GetOutletsMask().end(); ++it)
                {
                    auto [outletId, occupancy] = *it;
                    if (!occupancy)
                        continue;
                    if (outletId != -1)
                        doorwayValue[roomNode->GetCellIdentifier(it.cellIdx)] = 1.0f;
                    else
                        doorwayValue[roomNode->GetCellIdentifier(it.cellIdx)] = 0.8f;
                }
            }
    }


    double SimpleMovingStateGraph::CalculateExplorationValue(const RegionIdentifier& c)
    {
        // the exploration value is the sum of the uncertainty about the hit probability for all cells around (i,j), weighed by distance
        auto range = As<RoomNode>(c.node)->GetVisibilityMap().at(c.indices);

        double sum = 0;
        for (const auto& p : range)
        {
            float confidence = As<RoomNode>(c.node)->GetGasMap().dataAt(p).confidence;
            float distance = vmath::length(Vector2(c.indices - p)); // not the navigable distance, but we are close enough that it does not matter
            sum += confidence * std::exp(-distance * sigmaDist);
        }

        constexpr float baseInfo = 1e-0;
        return std::exp(-sum) + baseInfo;
    }
} // namespace GSL