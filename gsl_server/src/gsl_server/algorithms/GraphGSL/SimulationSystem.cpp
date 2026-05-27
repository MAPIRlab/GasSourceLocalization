#include "SimulationSystem.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include <stack>

namespace GSL::Graph_internal
{
    void SimulationSystem::Reset()
    {
        simulationCache.Clear();
        gasMapsWithRoomSource.clear();
        // blurMasks.clear(); //this can probably be retained (if the graph does not change)
    }

    SimWithResult SimulationSystem::SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> roomNode, Vector2 point)
    {
        SimWithResult result;
        result.hitMap = std::make_shared<std::vector<float>>(roomNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(point),
            .warmupAcceleration = options.warmupTimeAcc,
            .timesteps = options.iterationLimit,
            .deltaTime = options.deltaTime,
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = roomNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = roomNode->GetOutletsMask(),
                .exitsPerOutlet = std::vector<size_t>(roomNode->doorways.size(), 0),
                .numCellsOutlet = roomNode->GetOutletsCellCount(),
            }});
        // result.simulation->visibilityMap.emplace(roomNode->GetVisibilityMap());
        result.simulation->source.numFilamentsSecond = options.filamentsPerSecond;

        result.simulation->outlets->exitsPerOutlet.resize(roomNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(roomNode->doorways.size(), true);

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1);
        return result;
    }

    SimWithResult SimulationSystem::SimulateSingleRoomFromAABB(const std::shared_ptr<RoomNode> roomNode, AABB2D sourceAABB,
                                                               const std::set<std::shared_ptr<DoorwayNode>>& blockedDoorways)
    {
        SimWithResult result;
        Grid2DMetadata nodeMetadata = roomNode->GetOccupancy().metadata;
        Vector2 maxCoords = nodeMetadata.indicesToCoordinates(nodeMetadata.dimensions, false) - Vector2{0.001, 0.001};
        sourceAABB.min.x = std::clamp(sourceAABB.min.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.min.y = std::clamp(sourceAABB.min.y, nodeMetadata.origin.y, maxCoords.y);
        sourceAABB.max.x = std::clamp(sourceAABB.max.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.max.y = std::clamp(sourceAABB.max.y, nodeMetadata.origin.y, maxCoords.y);

        // configure the simulation
        result.hitMap = std::make_shared<std::vector<float>>(roomNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(sourceAABB),
            .warmupAcceleration = options.warmupTimeAcc,
            .timesteps = options.iterationLimit,
            .deltaTime = options.deltaTime,
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = roomNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = roomNode->GetOutletsMask(),
                .exitsPerOutlet = std::vector<size_t>(roomNode->doorways.size(), 0),
                .numCellsOutlet = roomNode->GetOutletsCellCount(),
            },
        });

        // result.simulation->visibilityMap.emplace(roomNode->GetVisibilityMap());
        result.simulation->source.numFilamentsSecond = options.filamentsPerSecond;

        result.simulation->outlets->exitsPerOutlet.resize(roomNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(roomNode->doorways.size(), true);

        for (size_t i = 0; i < roomNode->doorways.size(); i++)
            if (blockedDoorways.contains(roomNode->doorways.at(i)))
                result.simulation->outlets->enabled.at(i) = false;

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1);
        return result;
    }

    SimWithResult SimulationSystem::SimulateSingleRoomFromDoorway(const std::shared_ptr<const DoorwayNode> doorway)
    {
        auto roomNode = As<RoomNode>(doorway->from.lock());
        GSL_ASSERT_MSG(roomNode, "Tried to do simulation in place node which is not a room: {}", doorway->from.lock()->id);
        Grid2DMetadata nodeMetadata = roomNode->GetOccupancy().metadata;
        SimWithResult result = SimulateSingleRoomFromAABB(roomNode, doorway->aabb, doorway->samePhysicalDoorway);

        return result;
    }

    CompleteMap& SimulationSystem::SimulateEntireGraph(const std::shared_ptr<PlaceNode> firstNodeInSim, Vector2 sourcePoint)
    {
        // create an entry for this simulation in the results data structure
        mtx.lock();
        std::deque<CompleteMap>& maps = gasMapsWithRoomSource[firstNodeInSim];
        maps.push_back(CompleteMap{.source = std::make_shared<PointSource>(sourcePoint)});
        CompleteMap& map = maps.back();
        mtx.unlock();

        // run the simulation
        _SimulateEntireGraph(firstNodeInSim, map);
        return map;
    }

    CompleteMap& SimulationSystem::SimulateEntireGraph(std::shared_ptr<DoorwayNode> sourceDoorway)
    {
        // create an entry for this simulation in the results data structure
        mtx.lock();
        std::deque<CompleteMap>& maps = gasMapsWithRoomSource[sourceDoorway->to.lock()];
        maps.push_back(CompleteMap{.source = std::make_shared<DoorwaySource>(sourceDoorway)});
        CompleteMap& map = maps.back();
        mtx.unlock();

        // run the simulation
        _SimulateEntireGraph(sourceDoorway->to.lock(), map);
        return map;
    }

    void SimulationSystem::_SimulateEntireGraph(const std::shared_ptr<PlaceNode> firstNodeInSim, CompleteMap& completeGasMap)
    {
        // we need to get the final gas maps by combining the individual doorway simulations
        // this mainly means that we need to calculate the weights for the linear combination
        // given that a specific doorway may have been reached through more than one path in the graph, this is not a trivial task
        // we are going to do an exhaustive graph traversal, starting at the source node,
        // and we'll keep adding weight to the doorway based on how much gas reaches it through the specific path we are considering

        std::deque<NodeState> stateStack;
        if (Is<PointSource>(completeGasMap.source))
        {
            // handle the source node first, separately from the main traversal algorithm (since it doesn't have a doorway inlet)
            std::vector<float> weightDoorwaysfirstNodeInSim(firstNodeInSim->doorways.size(), 0);
            if (Is<RoomNode>(firstNodeInSim))
            {
                auto roomNode = As<RoomNode>(firstNodeInSim);
                SimWithResult result = SimulateSingleRoomFromPoint(roomNode, completeGasMap.source->GetPoint());
                completeGasMap.gasMaps[roomNode] = *result.hitMap;
                for (size_t i = 0; i < firstNodeInSim->doorways.size(); i++)
                    weightDoorwaysfirstNodeInSim.at(i) = result.ProportionInDoorway(i);
            }
            else // if the source is an outside node, just use the doorways themselves as the starting point
                for (size_t i = 0; i < firstNodeInSim->doorways.size(); i++)
                    weightDoorwaysfirstNodeInSim.at(i) = 1. / firstNodeInSim->doorways.size();

            for (size_t i = 0; i < firstNodeInSim->doorways.size(); i++)
            {
                const std::shared_ptr<DoorwayNode> doorway = firstNodeInSim->doorways.at(i);

                if (!Is<RoomNode>(doorway->to.lock()))
                    continue;

                NodeState nextRoom;
                nextRoom.doorSource = doorway->OtherSide();
                nextRoom.gasAtInlet = weightDoorwaysfirstNodeInSim.at(i);

                for (const auto& nextDoorway : doorway->to.lock()->doorways)
                    if (!nextRoom.doorSource->samePhysicalDoorway.contains(nextDoorway))
                        nextRoom.doorways.push(nextDoorway);

                stateStack.push_back(nextRoom);
            }
        }
        else
        {
            auto roomNode = As<RoomNode>(firstNodeInSim);
            if (roomNode)
            {
                completeGasMap.gasMaps[roomNode] = std::vector<float>(roomNode->GetOccupancy().data.size(), 1);
                for (size_t i = 0; i < roomNode->GetOccupancy().data.size(); i++)
                {
                    if (!roomNode->GetOccupancy().data.at(i))
                        completeGasMap.gasMaps[roomNode].at(i) = 0;
                }
            }

            auto source = As<DoorwaySource>(completeGasMap.source);
            NodeState nextRoom{.gasAtInlet = 1.f, .doorSource = source->doorway};
            for (const auto& nextDoorway : source->doorway->from.lock()->doorways)
                if (!nextRoom.doorSource->samePhysicalDoorway.contains(nextDoorway))
                    nextRoom.doorways.push(nextDoorway);
            stateStack.push_back(nextRoom);
        }

        // now, we start traversing the graph
        PropagateSimThroughGraph(stateStack, completeGasMap, firstNodeInSim);

        // post process the maps
        {
            // append all the hitmaps in completeGasMap.gasMaps
            std::vector<float> appendedHitMap;
            for (const auto& [node, map] : completeGasMap.gasMaps)
                std::ranges::transform(map, std::back_inserter(appendedHitMap), std::identity{});
            std::vector<Occupancy> appendedOccupancy;
            for (const auto& [node, map] : completeGasMap.gasMaps)
                std::ranges::transform(node->GetOccupancy().occupancy, std::back_inserter(appendedOccupancy), std::identity{});

            Utils::Winsorize(appendedHitMap, 5);
            Utils::PowerMaxNormalize(appendedHitMap, appendedOccupancy, options.normalizationPower);

            size_t globalIndex = 0;
            for (auto& [node, map] : completeGasMap.gasMaps)
                for (size_t i = 0; i < map.size(); i++)
                    map.at(i) = appendedHitMap.at(globalIndex++);

            mtx.lock();
            for (auto& [node, map] : completeGasMap.gasMaps)
                Simulation::blurHitMap(map, options.blurSigma, node->GetOccupancy(), blurMasks[node]);
            mtx.unlock();

            // // normalize by the global maximum!
            float max = 0;
            for (const auto& [node, map] : completeGasMap.gasMaps)
            {
                auto max_it = std::max_element(map.begin(), map.end());
                float localMax = *max_it;
                GSL_ASSERT(localMax == 0 || node->GetOccupancy().occupancy.at(std::distance(map.begin(), max_it)));
                max = std::max(max, localMax);
            }

            for (auto& [node, map] : completeGasMap.gasMaps)
            {
                for (size_t i = 0; i < map.size(); i++)
                    map.at(i) /= max;
            }
        }
    }

    void SimulationSystem::PropagateSimThroughGraph(std::deque<NodeState>& stateStack, CompleteMap& completeGasMap, const std::shared_ptr<PlaceNode> firstNodeInSim)
    {
        std::map<std::shared_ptr<const DoorwayNode>, float> totalGasThroughDoorway;

        for (const auto& node : stateStack)
        {
            totalGasThroughDoorway[node.doorSource] = node.gasAtInlet;
        }

        constexpr float minimumGasThr = 1e-6;
        while (!stateStack.empty())
        {
            ZoneScopedN("Graph propagation");
            NodeState& current = stateStack.back();

            // if we cannot keep expanding this node, pop it from the stack
            if (current.gasAtInlet < minimumGasThr || current.doorways.empty())
                stateStack.pop_back();
            else
            {
                // otherwise, let's get the next doorway and continue
                NodeState next;
                next.doorSource = current.doorways.top()->OtherSide();
                current.doorways.pop();
                if (!Is<RoomNode>(next.doorSource->from))
                    continue;

                SimWithResult result = simulationCache.Get(current.doorSource);

                // adjust for the fact that the normalized concentration at the inlet might not be 1
                float concentrationInlet = result.ProportionInDoorway(current.doorSource->GetIndex());
                float weight = 1.f / concentrationInlet;

                // calculate how much of the gas in the current node makes it to the next node
                size_t outletIndex = next.doorSource->OtherSide()->GetIndex();
                float gasProportion = weight * result.ProportionInDoorway(outletIndex);

                // if no gas exits this room at all (a dead end or other weird edge case), just stop expansion in this direction
                if (gasProportion == 0 || !std::isfinite(gasProportion))
                    continue;

                next.gasAtInlet = current.gasAtInlet * gasProportion;
                if (next.gasAtInlet < minimumGasThr)
                    continue;

                // update the total amount of gas that passes through the doorway
                totalGasThroughDoorway[next.doorSource] += next.gasAtInlet;

                // fill in the doorways of the next state node
                for (const auto nextDoorway : next.doorSource->from.lock()->doorways)
                    if (!next.doorSource->samePhysicalDoorway.contains(nextDoorway))
                        next.doorways.push(nextDoorway);

                // push the new state on top
                stateStack.push_back(next);
            }
        }

        // OK, now we've done all that, we can combine the individual simulation maps,
        // weighted by the amount of gas that should have passed through each doorway
        for (const auto& node : graph->nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room || node == firstNodeInSim)
                continue;

            completeGasMap.gasMaps[room] = std::vector<float>(room->GetOccupancy().data.size(), 0.);
            for (const auto& doorway : node->doorways)
            {
                if (!totalGasThroughDoorway.contains(doorway) || !simulationCache.Contains(doorway))
                    continue;

                const SimWithResult& result = simulationCache.Get(doorway);
                float weight = totalGasThroughDoorway.at(doorway);

                // adjust for the fact that the normalized concentration at the inlet might not be 1
                float concentrationInlet = result.ProportionInDoorway(doorway->GetIndex());
                weight /= concentrationInlet;

                for (size_t i = 0; i < result.hitMap->size(); i++)
                    completeGasMap.gasMaps[room].at(i) += result.hitMap->at(i) * weight;
            }
        }
    }

    MarkerArray SimulationSystem::VisualizeCachedResults(std::shared_ptr<PlaceNode> sourceRoom, size_t simulationIndex, float nodeSeparationViz)
    {
        if (!gasMapsWithRoomSource.contains(sourceRoom))
            return MarkerArray{};

        auto& gasMaps = gasMapsWithRoomSource.at(sourceRoom);
        if (simulationIndex >= gasMaps.size())
            return MarkerArray{};

        CompleteMap& map = gasMaps.at(simulationIndex);
        return VisualizeCompleteMap(map, graph->nodes, nodeSeparationViz, 0.1);
    }

    SimWithResult SimulationSystem::SimulationCache::Get(std::shared_ptr<const DoorwayNode> doorway)
    {
        if (SyncContains(simulations, doorway))
            return simulations.at(doorway);

        if (SyncContains(simsInFlight, doorway))
        {
            while (SyncContains(simsInFlight, doorway))
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            return simulations.at(doorway);
        }
        else
        {
            // run the simulation and add it to the cache
            {
                std::scoped_lock lock(mtx);
                simsInFlight.insert(doorway);
                GSL_INFO("Running simulation in room {} from doorway {}", doorway->from.lock()->id, doorway->GetName());
            }
            SimWithResult result = simSys->SimulateSingleRoomFromDoorway(doorway);
            {
                std::scoped_lock lock(mtx);
                simulations[doorway] = result;
                simsInFlight.erase(doorway);
            }
            return result;
        }
    }

    void SimulationSystem::SimulationCache::Clear()
    {
        simulations.clear();
        simsInFlight.clear();
    }

    bool SimulationSystem::SimulationCache::IsRunning(const std::shared_ptr<const DoorwayNode> doorway)
    {
        return SyncContains(simsInFlight, doorway);
    }

    template <typename T, typename U>
    bool SimulationSystem::SimulationCache::SyncContains(const T& collection, const U& element)
    {
        std::scoped_lock lock(mtx);
        return collection.contains(element);
    }

    void SimulationSystem::blurTest(std::vector<Vector2Int> points)
    {
        size_t sizeX = 25;
        size_t sizeY = 25;

        std::vector<float> map(sizeX * sizeY, 0);
        for (const auto& point : points)
        {
            map.at(sizeY * point.y + point.x) += 1; // Set the center cell to 1
        }

        std::vector<Occupancy> occupancy_data(sizeX * sizeY, Occupancy::Free);

        Grid2DMetadata metadata = Grid2DMetadata{.dimensions = Vector2Int(sizeX, sizeY)};
        Grid2D<Occupancy> occupancy{occupancy_data, occupancy_data, metadata};

        std::optional<SimulationBlurMask> mask = std::nullopt;
        Simulation::blurHitMap(map, 1, occupancy, mask);

        float total = std::accumulate(map.begin(), map.end(), 0.0f);
        GSL_INFO("Total gas: {}", total);
    }

} // namespace GSL::Graph_internal