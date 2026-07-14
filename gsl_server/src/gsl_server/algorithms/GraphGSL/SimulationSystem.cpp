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
        emergencyStopped = false;
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
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1.f);
        GSL_ASSERT(std::all_of(result.hitMap->begin(), result.hitMap->end(), [](float f)
                               {
                                   return std::isfinite(f);
                               }));
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

        // equate the concentration at all the inlet cells to avoid having an artifact on the back ranks
        AABB2DInt aabbIndices{nodeMetadata.coordinatesToIndices(sourceAABB.min), nodeMetadata.coordinatesToIndices(sourceAABB.max)};
        Grid2D<float> resultGrid(*result.hitMap, roomNode->GetOccupancy());
        float max = 0;
        for (Vector2Int cell : aabbIndices)
            max = std::max(max, resultGrid.dataAt(cell));
        for (Vector2Int cell : aabbIndices)
            if (roomNode->GetOccupancy().occupancyAt(cell))
                resultGrid.dataAt(cell) = max;

        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1.f);
        GSL_ASSERT(std::all_of(result.hitMap->begin(), result.hitMap->end(), [](float f)
                               {
                                   return std::isfinite(f);
                               }));
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

    void SimulationSystem::EmergencyStop()
    {
        emergencyStopped = true;
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
                nextRoom.gasProportion = nextRoom.gasAtInlet;

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
            NodeState nextRoom{.gasAtInlet = 1.f, .gasProportion = 1.f, .doorSource = source->doorway};
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

            Utils::Winsorize(appendedHitMap, 2);
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

            // should not happen, but avoid NaNs just in case
            if(max == 0)
                max = 1.f;

            for (auto& [node, map] : completeGasMap.gasMaps)
            {
                for (size_t i = 0; i < map.size(); i++)
                    map.at(i) /= max;
            }
        }
    }

    void SimulationSystem::PropagateSimThroughGraph(std::deque<NodeState>& initialNodeStates, CompleteMap& completeGasMap, const std::shared_ptr<PlaceNode> firstNodeInSim)
    {
        std::map<std::shared_ptr<const DoorwayNode>, float> totalGasThroughDoorway;

        for (const auto& node : initialNodeStates)
        {
            totalGasThroughDoorway[node.doorSource] = node.gasAtInlet;
        }

        std::map<std::shared_ptr<const DoorwayNode>, GraphCacheEntry> graphCache;

        constexpr float minimumGasProportion = 1e-2;
        constexpr float minimumGasInlet = 1e-6;
        size_t iterations = 0;
        std::deque<NodeState> stateStack;

#define LOG_DETAILS 1
#if LOG_DETAILS
#define LOG_TRACE(...) GSL_INFO(__VA_ARGS__)
#else
#define LOG_TRACE(...)
#endif

        for (const auto& initialNode : initialNodeStates)
        {
            stateStack.push_back(initialNode);
            while (!stateStack.empty())
            {
                ZoneScopedN("Graph propagation");

                if (emergencyStopped)
                    break;

                NodeState& current = stateStack.back();
#if LOG_DETAILS
                std::stringstream ss;
                for (const auto& node : stateStack)
                    ss << fmt::format("->({:.2e}){} ", node.gasAtInlet, node.doorSource->GetDebuggingName());
                GSL_INFO("{}", ss.str(), current.doorSource->GetDebuggingName());
#endif
                // loop detection and handling
                {
                    // // find a state further in the stack which refers to the same doorway as the current one
                    auto loopIterator = std::find_if(stateStack.rbegin() + 1, // +1 to avoid the current state itself
                                                     stateStack.rend(),
                                                     [&current](const NodeState& n)
                                                     {
                                                         return n.doorSource == current.doorSource;
                                                     });
                    bool loopDetected = loopIterator != stateStack.rend();
                    if (loopDetected)
                    {
#if LOG_DETAILS
                        std::stringstream ss;
                        for (const auto& node : stateStack)
                        {
                            ss << fmt::format("{} -> ({:.2e})", node.doorSource->GetDebuggingName(), node.gasAtInlet);
                        }
                        GSL_INFO("Loop detected: {}{}", ss.str(), current.doorSource->GetDebuggingName());
#endif

                        NodeState& previous = *loopIterator;
                        float loopedProportion = current.gasAtInlet / previous.gasAtInlet;

                        if (loopedProportion >= 1)
                        {
                            GSL_ERROR("Loop detected with proportion >= 1. This is a failure of the model assumptions. "
                                      "Forcefully setting to 0.9 to avoid infinite gas!");
                            loopedProportion = 0.9;
                            // GSL_ASSERT(false); // catch the debugger, if present
                        }

                        // since this is a loop, it is technically an infinite series: 1 + x + x*x...
                        // it converges (as long as x < 1), to this:
                        float seriesTotal = 1.f / (1 - loopedProportion);

                        // update what proportion of the initial amount makes it to each doorway in the cache
                        // this makes it so the next time we reach the doorway at the loop start we correctly take the looping into consideration
                        // from 1 -> x  to  total -> total * x
                        for (auto it = loopIterator; it != stateStack.rend(); it++)
                        {
                            for (auto& [doorway, gasProportion] : graphCache[it->doorSource].gasProportion)
                            {
                                // if the doorway is reachable from the loop start, update its gasProportion in *all* caches
                                if (graphCache[loopIterator->doorSource].gasProportion.contains(doorway))
                                    gasProportion *= seriesTotal;
                            }
                        }

                        // now, update the loop start itself
                        // this always had an implied proportion of 1 (because it is the reference value)
                        // so we have to add it to the cache as total - 1
                        // since it is *technically* possible to have more than one loop, we add the value rather than just setting it
                        float extraProportion = seriesTotal - 1;
                        graphCache[current.doorSource].gasProportion[current.doorSource] += extraProportion;
                        float extraAmount = extraProportion * previous.gasAtInlet;

                        // then, we update the actual gas amounts in the result data structure, (not the cached proportions)
                        for (auto& [doorway, gasProportion] : graphCache[current.doorSource].gasProportion)
                            totalGasThroughDoorway[doorway] += gasProportion * extraAmount;

                        // we need to remove the (first iteration)looped gas fraction from the loop start doorway,
                        // as that has already been counted when creating currentState
                        totalGasThroughDoorway[current.doorSource] -= current.gasAtInlet;

                        // finally, update the NodeStates for all the doorways in the loop, in case they still have some unexplored doorways
                        // (which should still consider the extra gas due to the loop)
                        for (auto it = stateStack.rbegin(); it != loopIterator; it++)
                        {
                            NodeState& node = *it;
                            float gasProportion = graphCache[current.doorSource].gasProportion[node.doorSource];
                            node.gasAtInlet += gasProportion * extraAmount;
                        }
                        previous.gasAtInlet += extraAmount;

                        // TODO currently, we are not using the "complete node" aspect of the cache:
                        // nodes are expanded every time you reach them
                        // the reason is that that mechanism causes quite a few problems when considering loops
                        // I would love to figure out how to make it work (incomplete attempt below), but cannot currently dedicate more time to it

                        // update the caches for every node in the loop:
                        // it does not matter where the loop starts (it is a loop, after all), so if we reach it at a different spot during a different
                        // path expansion, the cache of that new "loop start" node must reflect all the same calculations we have already done
                        // float incompleteLoopAmount = seriesTotal; // how much of the gas that enters the gas at an arbitrary node makes it to the loop start
                        //                                           // (where the start is just the one used for the calculations above) on the *first* go-around
                        // for (auto it = stateStack.rbegin(); it != loopIterator; it++)
                        // {
                        //     NodeState& node = *it;
                        //     incompleteLoopAmount *= node.gasProportion;
                        //     // we are going to add all the nodes that are in the cache of the loop start to the cache of every other node in the loop
                        //     // importantly, scaled by how much of the gas makes it to the "loop start" node if the loop actually starts somewhere else
                        //     for (auto& [doorway, gasProportion] : graphCache[loopIterator->doorSource].gasProportion)
                        //     {
                        //         graphCache[node.doorSource].gasProportion[doorway] += gasProportion * incompleteLoopAmount;
                        //     }
                        // }

                        stateStack.pop_back();
                        continue;
                    }
                }

                // using the graph cache to avoid re-treading paths that have already been completely explored
                // this is currently not used because it causes some complications with the loop handling
                // if (graphCache[current.doorSource].complete)
                // {
                //     LOG_TRACE("Doorway {} is complete!", current.doorSource->GetDebuggingName());
                //     for (auto& [doorway, gasProportion] : graphCache[current.doorSource].gasProportion)
                //     {
                //         float gasThisDoorway = gasProportion * current.gasAtInlet;
                //         totalGasThroughDoorway[doorway] += gasThisDoorway;
                //         LOG_TRACE("Adding {} gas to {}", gasThisDoorway, doorway->GetName());
                //         // add this cached node to all the caches of nodes in the path so far!
                //         for (auto& state : stateStack)
                //             graphCache[state.doorSource].gasProportion[doorway] += gasThisDoorway / state.gasAtInlet;
                //     }
                //     stateStack.pop_back();
                //     continue;
                // }

                // TODO if we re-activate the "complete" check, remove the inlet gas condition! the cache is not reliable if we don't actually exhaust the path the first time we reach it
                if (current.gasProportion <= minimumGasProportion || current.gasAtInlet <= minimumGasInlet)
                {
                    LOG_TRACE("Pruning {}, too little gas", current.doorSource->GetDebuggingName());
                    stateStack.pop_back();
                    continue;
                }

                if (current.doorways.empty())
                {
                    LOG_TRACE("Marking {} complete", current.doorSource->GetDebuggingName());
                    graphCache[current.doorSource].complete = true;
                    stateStack.pop_back();
                    continue;
                }

                // otherwise, let's get the next doorway and continue
                NodeState next;
                next.doorSource = current.doorways.top()->OtherSide();
                current.doorways.pop();
                if (!Is<RoomNode>(next.doorSource->from))
                {
                    LOG_TRACE("Skipping outside node {}", next.doorSource->from.lock()->id);
                    continue;
                }

                float gasProportion;
                DoorwayPair pair{current.doorSource, next.doorSource};
                if (Utils::SyncedAccess(doorwayPairs).Get().contains(pair))
                    gasProportion = Utils::SyncedAccess(doorwayPairs).Get().at(pair);
                else
                {
                    SimWithResult result = simulationCache.Get(current.doorSource);

                    // adjust for the fact that the normalized concentration at the inlet might not be 1
                    float concentrationInlet = result.ProportionInDoorway(current.doorSource->GetIndex());
                    float weight = 1.f / concentrationInlet;

                    // calculate how much of the gas in the current node makes it to the next node
                    size_t outletIndex = next.doorSource->OtherSide()->GetIndex();
                    gasProportion = weight * result.ProportionInDoorway(outletIndex);
                }

                next.gasProportion = gasProportion;
                next.gasAtInlet = current.gasAtInlet * gasProportion;
                if (next.gasProportion < minimumGasProportion || !std::isfinite(next.gasAtInlet))
                    continue;

                // update the total amount of gas that passes through the doorway
                totalGasThroughDoorway[next.doorSource] += next.gasAtInlet;

                for (auto& previous : stateStack)
                    graphCache[previous.doorSource].gasProportion[next.doorSource] += next.gasAtInlet / previous.gasAtInlet;

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
            if (!room)
                continue;

            if (room != firstNodeInSim)
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
                GSL_TRACE("Running simulation in room {} from doorway {}", doorway->from.lock()->id, doorway->GetName());
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