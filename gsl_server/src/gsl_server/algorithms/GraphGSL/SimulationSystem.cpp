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
        ScopedStopwatch s("sims");
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
            },
        });
        result.simulation->source.numFilamentsSecond = options.filamentsPerSecond;

        result.simulation->outlets->exitsPerOutlet.resize(roomNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(roomNode->doorways.size(), true);

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        Utils::Windsorize(*result.hitMap, 5);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, options.normalizationPower);
        Simulation::blurHitMap(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1);
        return result;
    }

    SimWithResult SimulationSystem::SimulateSingleRoomFromAABB(const std::shared_ptr<RoomNode> roomNode, AABB2D sourceAABB,
                                                               const std::set<const DoorwayNode*>& blockedDoorways)
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

        result.simulation->source.numFilamentsSecond = options.filamentsPerSecond;

        result.simulation->outlets->exitsPerOutlet.resize(roomNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(roomNode->doorways.size(), true);

        for (size_t i = 0; i < roomNode->doorways.size(); i++)
            if (blockedDoorways.contains(&roomNode->doorways.at(i)))
                result.simulation->outlets->enabled.at(i) = false;

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        if (options.cummulativeMap)
        {
            Utils::Windsorize(*result.hitMap, 5);
            result.maxBeforeNormalize = *std::max_element(result.hitMap->begin(), result.hitMap->end());
            Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, options.normalizationPower);
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);
            Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1);
        }
        else
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);
        return result;
    }

    SimWithResult SimulationSystem::SimulateSingleRoomFromDoorway(const DoorwayNode& doorway)
    {
        auto roomNode = As<RoomNode>(doorway.from.lock());
        GSL_ASSERT_MSG(roomNode, "Tried to do simulation in place node which is not a room: {}", doorway.from.lock()->id);
        Grid2DMetadata nodeMetadata = roomNode->GetOccupancy().metadata;
        SimWithResult result = SimulateSingleRoomFromAABB(roomNode, doorway.aabb, {&doorway});

        return result;
    }

    void SimulationSystem::SimulateEntireGraph(const std::shared_ptr<PlaceNode> sourceNode, Vector2 sourcePoint)
    {
        // create an entry for this simulation in the results data structure
        mtx.lock();
        std::deque<CompleteMap>& maps = gasMapsWithRoomSource[sourceNode];
        maps.push_back(CompleteMap{.sourcePoint = sourcePoint});
        mtx.unlock();

        // run the simulation
        _SimulateEntireGraph(sourceNode, maps.back());
    }

    void SimulationSystem::_SimulateEntireGraph(const std::shared_ptr<PlaceNode> sourceNode, CompleteMap& completeGasMap)
    {
        std::map<const DoorwayNode*, float> totalGasThroughDoorway;

        // we need to get the final gas maps by combining the individual doorway simulations
        // this mainly means that we need to calculate the weights for the linear combination
        // given that a specific doorway may have been reached through more than one path in the graph, this is not a trivial task
        // we are going to do an exhaustive graph traversal, starting at the source node,
        // and we'll keep adding weight to the doorway based on how much gas reaches it through the specific path we are considering
        struct NodeState
        {
            float gasAtInlet;
            const DoorwayNode* doorSource;
            std::stack<const DoorwayNode*> doorways;
        };
        std::stack<NodeState> stateStack;

        // handle the source node first, separately from the main traversal algorithm (since it doesn't have a doorway inlet)
        std::vector<float> weightDoorwaysSourceNode(sourceNode->doorways.size(), 0);
        if (Is<RoomNode>(sourceNode))
        {
            auto roomNode = As<RoomNode>(sourceNode);
            SimWithResult result = SimulateSingleRoomFromPoint(roomNode, completeGasMap.sourcePoint);
            completeGasMap.gasMaps[roomNode] = *result.hitMap;
            for (size_t i = 0; i < sourceNode->doorways.size(); i++)
                weightDoorwaysSourceNode.at(i) = result.ProportionInDoorway(i);
        }
        else // if the source is an outside node, just use the doorways themselves as the starting point
            for (size_t i = 0; i < sourceNode->doorways.size(); i++)
                weightDoorwaysSourceNode.at(i) = 1. / sourceNode->doorways.size();

        for (size_t i = 0; i < sourceNode->doorways.size(); i++)
        {
            const DoorwayNode& doorway = sourceNode->doorways.at(i);

            if (!Is<RoomNode>(doorway.to))
                continue;

            NodeState nextRoom;
            nextRoom.doorSource = &doorway.OtherSide();
            nextRoom.gasAtInlet = weightDoorwaysSourceNode.at(i);

            GSL_INFO("{}->{}   -   {}", doorway.from.lock()->id, doorway.to.lock()->id, nextRoom.gasAtInlet);

            for (const auto& nextDoorway : doorway.to.lock()->doorways)
                if (&nextDoorway != nextRoom.doorSource)
                    nextRoom.doorways.push(&nextDoorway);

            totalGasThroughDoorway[nextRoom.doorSource] = nextRoom.gasAtInlet;
            stateStack.push(nextRoom);
        }

        // now, we start traversing the graph

        constexpr float minimumGasThr = 1e-1;
        while (!stateStack.empty())
        {
            NodeState& current = stateStack.top();

            // if we cannot keep expanding this node, pop it from the stack
            if (current.gasAtInlet < minimumGasThr || current.doorways.empty())
                stateStack.pop();
            else
            {
                // parallelization optimization: if this simulation is already running on another thread,
                // move this state to the back of the queue and keep going on a different direction
                // if (simulationCache.IsRunning(current.doorSource))
                // {
                //     stateQueue.push(current);
                //     stateQueue.pop();
                //     continue;
                // }

                // otherwise, let's get the next doorway and continue
                NodeState next;
                next.doorSource = &current.doorways.top()->OtherSide();
                current.doorways.pop();
                if (!Is<RoomNode>(next.doorSource->from))
                    continue;

                GSL_INFO("{}->{}", current.doorSource->to.lock()->id, current.doorSource->from.lock()->id);

                SimWithResult result = simulationCache.Get(current.doorSource);

                // if no gas exits this room at all (a dead end or other weird edge case), just stop expansion in this direction
                if (result.simulation->outlets->totalExitCount == 0)
                {
                    stateStack.pop();
                    continue;
                }

                // adjust for the fact that the normalized concentration at the inlet might not be 1
                float concentrationInlet = result.ProportionInDoorway(current.doorSource->GetIndex());
                float weight = 1.f / concentrationInlet;

                // calculate how much of the gas in the current node makes it to the next node
                size_t outletIndex = next.doorSource->OtherSide().GetIndex();
                float gasProportion = weight * result.ProportionInDoorway(outletIndex);

                next.gasAtInlet = current.gasAtInlet * gasProportion;
                GSL_INFO("Remaining: {}", next.gasAtInlet);

                // update the total amount of gas that passes through the doorway
                totalGasThroughDoorway[next.doorSource] += next.gasAtInlet;

                // fill in the doorways of the next state node
                for (const auto& nextDoorway : next.doorSource->from.lock()->doorways)
                    if (&nextDoorway != next.doorSource)
                        next.doorways.push(&nextDoorway);

                // push the new state on top
                stateStack.push(next);
            }
        }

        // OK, now we've done all that, we can combine the individual simulation maps,
        // weighted by the amount of gas that should have passed through each doorway

        for (const auto& node : graph->nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room || node == sourceNode)
                continue;

            completeGasMap.gasMaps[room] = std::vector<float>(room->GetOccupancy().data.size(), 0.);
            for (const auto& doorway : node->doorways)
            {
                if (!totalGasThroughDoorway.contains(&doorway) || !simulationCache.Contains(&doorway))
                    continue;

                const SimWithResult& result = simulationCache.Get(&doorway);
                float weight = totalGasThroughDoorway.at(&doorway);

                // adjust for the fact that the normalized concentration at the inlet might not be 1
                float concentrationInlet = result.ProportionInDoorway(doorway.GetIndex());
                weight /= concentrationInlet;

                for (size_t i = 0; i < result.hitMap->size(); i++)
                    completeGasMap.gasMaps[room].at(i) += result.hitMap->at(i) * weight;
            }
        }

        // normalize by the global maximum!
        float max = 0;
        for (const auto& [node, map] : completeGasMap.gasMaps)
        {
            float localMax = *std::max_element(map.begin(), map.end());
            max = std::max(max, localMax);
        }

        for (auto& [node, map] : completeGasMap.gasMaps)
        {
            for (size_t i = 0; i < map.size(); i++)
                map.at(i) /= max;
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

    SimWithResult SimulationSystem::SimulationCache::Get(const DoorwayNode* doorway)
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
            }
            SimWithResult result = simSys->SimulateSingleRoomFromDoorway(*doorway);
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

    bool SimulationSystem::SimulationCache::IsRunning(const DoorwayNode* doorway)
    {
        return SyncContains(simsInFlight, doorway);
    }

    template <typename T, typename U>
    bool SimulationSystem::SimulationCache::SyncContains(const T& collection, const U& element)
    {
        std::scoped_lock lock(mtx);
        return collection.contains(element);
    }

} // namespace GSL::Graph_internal