#include "MovingStateGraph.hpp"
#include "GraphGSL.hpp"

#define SCALE_EXPECTED_MAPS 0

namespace GSL
{
    MovingStateGraph::MovingStateGraph(Algorithm* alg) : ManualNavigationState(alg), gsl(dynamic_cast<GraphGSL*>(alg))
    {
        // init the variance arrays
        for (auto node : gsl->graph.nodes)
            if (auto roomNode = As<RoomNode>(node))
            {
                expectedGasVariances[roomNode].resize(roomNode->GetOccupancy().data.size());
                finalInfoValue[roomNode].resize(roomNode->GetOccupancy().data.size());

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

    void MovingStateGraph::OnEnterState(State* previous)
    {
        UpdateInfoGain();
        ManualNavigationState::OnEnterState(previous);
    }

    void MovingStateGraph::chooseGoalAndMove()
    {
    }

    void MovingStateGraph::UpdateInfoGain()
    {
        ScopedStopwatch watch("UpdateInfoGain");
        MultiGrid<KernelDMVW::KernelCell> kernelCells = gsl->graph.GetAllKernelCells();
        for (auto node : gsl->graph.nodes)
        {
            auto roomNode = As<RoomNode>(node);
            if (!roomNode)
                continue;

            for (size_t i = 0; i < roomNode->GetGasMap().data.size(); ++i)
            {
                if (!roomNode->GetGasMap().occupancy.at(i))
                    continue;
                CellIdentifier id = roomNode->GetCellIdentifier(i);
                explorationValue[id] = CalculateExplorationValue(id);
                finalInfoValue[roomNode].at(i) = explorationValue[id]                                    //
                                                 * (expectedGasVariances[roomNode].at(i).variance + 0.1) //
                                                                                                         //  * doorwayValue[id]                                      //
                    ;
            }
        }
    }

    void MovingStateGraph::ResetVariances()
    {
        expectedGasVariances.clear();
    }

    double MovingStateGraph::CalculateExplorationValue(const CellIdentifier& c)
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

    void MovingStateGraph::UpdateExpectedVariance()
    {
        GSL_INFO("Updating information gain");
        ScopedStopwatch watch("info gain");
        std::vector<CellIdentifier> allFreeCells = gsl->graph.GetAllFreeCells();

        // reset all the information from previous simulations
        expectedGasVariances.clear();

        // start updating the values with the latest results

        auto updateWithExpectedMap = [&](const Graph_internal::CompleteMap& completeMap, float probability)
        {
            for (const auto& entry : completeMap.gasMaps)
            {
                const auto& room = entry.first;
                const auto& map = entry.second;
                if (!expectedGasVariances.contains(room))
                    expectedGasVariances[room] = std::vector<Utils::RunningVariance>(room->GetOccupancy().data.size());

#pragma omp parallel for schedule(dynamic, 50)
                for (size_t i = 0; i < map.size(); ++i)
                {
                    if (!room->GetOccupancy().data.at(i))
                        continue;
                    float value = map.at(i);
                    float weight = std::pow(probability * 100, 2);
                    expectedGasVariances[room].at(i).Update(value, weight);
                    GSL_ASSERT(std::isfinite(expectedGasVariances[room].at(i).variance));
                }
            }
        };

        for (auto node : gsl->graph.nodes)
        {
            if (expectedGasMaps.contains(node->GetNodeIdentifier()))
            {
                float probability = gsl->roomSourceProbabilities.at(node);
                Graph_internal::CompleteMap& completeMap = *expectedGasMaps.at(node->GetNodeIdentifier()).map;
                updateWithExpectedMap(completeMap, probability);
            }
            else
            {
                auto room = As<RoomNode>(node);
                if (!room)
                    continue;
                Grid2D<float> probabilities = room->GetSourceProbabilities();
                for (size_t cellIndex = 0; cellIndex < probabilities.data.size(); cellIndex++)
                {
                    float probability = probabilities.data.at(cellIndex);
                    if (probability < 1e-7)
                        continue;

                    CellIdentifier sourceID = room->GetCellIdentifier(cellIndex);
                    Graph_internal::CompleteMap& completeMap = *expectedGasMaps.at(sourceID).map;
                    updateWithExpectedMap(completeMap, probability);
                }
            }
        }
    }

    void MovingStateGraph::UpdateExpectedGasGeometricLevel(std::mutex& mtx, CellIdentifier id, float scale, const Graph_internal::CompleteMap& map)
    {
        {
            std::scoped_lock lock(mtx);
            expectedGasMaps[id] = {.map = std::make_shared<Graph_internal::CompleteMap>()};
        }

        // store the (scaled?) result for movement strategy
        for (const auto& [room, localSimMap] : map.gasMaps)
        {
            expectedGasMaps[id].map->gasMaps[room].resize(localSimMap.size(), 0);
#if SCALE_EXPECTED_MAPS
            for (size_t i = 0; i < localSimMap.size(); i++)
                expectedGasMaps[id].map->gasMaps[room].at(i) = localSimMap.at(i) * scale;
#else
            for (size_t i = 0; i < localSimMap.size(); i++)
                expectedGasMaps[id].map->gasMaps[room].at(i) = localSimMap.at(i);
#endif
        }
    }

    void MovingStateGraph::AssignAABBGasMapToCell(CellIdentifier aabbID, CellIdentifier cellID)
    {
        expectedGasMaps[cellID] = expectedGasMaps.at(aabbID);
    }

    void MovingStateGraph::UpdateExpectedGasRoomLevel(std::shared_ptr<PlaceNode> sourceNode,
                                                      const std::vector<double>& scales,
                                                      const std::deque<Graph_internal::CompleteMap>& simulations)
    {
        // store the scaled sum of the simulation results, to later evaluate the most interesting points for future measurement
        CellIdentifier id{.node = sourceNode, .indices = CellIdentifier::WHOLE_NODE};
        expectedGasMaps[id] = {.map = std::make_shared<Graph_internal::CompleteMap>()};

        float scaleSum = std::accumulate(scales.begin(), scales.end(), 0.0f);
        size_t simIndex = 0;
        for (const Graph_internal::CompleteMap& simulation : simulations)
        {
            for (const auto& [room, localSimMap] : simulation.gasMaps)
            {
                if (room == sourceNode)
                    continue;
                if (!expectedGasMaps[id].map->gasMaps.contains(room))
                    expectedGasMaps[id].map->gasMaps[room].resize(localSimMap.size(), 0);
#if SCALE_EXPECTED_MAPS
                for (size_t i = 0; i < localSimMap.size(); i++)
                    expectedGasMaps[id].map->gasMaps[room].at(i) += scales.at(simIndex) * localSimMap.at(i);
#else
                float scale = (scales.at(simIndex) / scaleSum);
                if (!std::isfinite(scale))
                    scale = 0;
                for (size_t i = 0; i < localSimMap.size(); i++)
                    expectedGasMaps[id].map->gasMaps[room].at(i) += scale * localSimMap.at(i);
#endif
            }

            simIndex++;
        }
    }

    MarkerArray MovingStateGraph::VisualizeInfoGain()
    {
        MarkerArray array;
        size_t id = 0;

        for (auto node : gsl->graph.nodes)
        {
            if (!Is<RoomNode>(node))
                continue;
            auto roomNode = As<RoomNode>(node);
            Grid2DMetadata vizMetadata = roomNode->GetSourceProbabilities().metadata;
            vizMetadata.origin = vizMetadata.origin * gsl->graph.vizOptions.nodeSeparationViz;

            Grid2D<float> grid(finalInfoValue.at(roomNode), roomNode->GetSourceProbabilities().occupancy, vizMetadata);
            Marker marker = Utils::createPointsMarker(grid, 0, gsl->graph.vizOptions.maxInfoGain, Utils::ValueColorMode::Linear, Utils::Colors::ColorMaps::Jet, 0.3);
            marker.id = id++;
            array.markers.push_back(marker);
        }

        return array;
    }

    void MovingStateGraph::AutoSetMaxInfoGain()
    {
        float maxInfoGain = 0;
        for (const auto& node : gsl->graph.nodes)
            if (auto room = As<RoomNode>(node))
            {
                auto localMax = std::max_element(finalInfoValue[room].begin(),
                                                 finalInfoValue[room].end());
                maxInfoGain = std::max<float>(maxInfoGain, *localMax);
            }
        gsl->graph.vizOptions.maxInfoGain = maxInfoGain;
    }
} // namespace GSL