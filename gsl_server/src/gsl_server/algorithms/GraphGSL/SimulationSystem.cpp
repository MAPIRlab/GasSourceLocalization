#include "SimulationSystem.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include <stack>

namespace GSL::Graph_internal
{
    void SimulationSystem::Reset()
    {
        simulationCache.clear();
        gasMapsWithRoomSource.clear();
        // blurMasks.clear(); //this can probably be retained (if the graph does not change)
    }

    SimulationSystem::SimWithResult SimulationSystem::SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> roomNode, Vector2 point)
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
            },
        });

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

    SimulationSystem::SimWithResult SimulationSystem::SimulateSingleRoomFromAABB(const std::shared_ptr<RoomNode> roomNode, AABB2D sourceAABB,
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

    SimulationSystem::SimWithResult SimulationSystem::SimulateSingleRoomFromDoorway(const DoorwayNode& doorway)
    {
        auto roomNode = As<RoomNode>(doorway.from.lock());
        GSL_ASSERT_MSG(roomNode, "Tried to do simulation in place node which is not a room: {}", doorway.from.lock()->id);
        Grid2DMetadata nodeMetadata = roomNode->GetOccupancy().metadata;
        SimWithResult result = SimulateSingleRoomFromAABB(roomNode, doorway.aabb, {&doorway});

        // store the simulation result in the cache
        //-------------------
        simulationCache[&doorway] = result;

        return result;
    }

    void SimulationSystem::SimulateEntireGraphFromRoom(const Graph& graph, const std::shared_ptr<RoomNode> sourceNode)
    {
        CompleteMap& completeGasMap = gasMapsWithRoomSource[sourceNode];
        std::map<const DoorwayNode*, float> totalGasThroughDoorway;

        // now, we need to get the final gas maps by combining the individual doorway simulations
        // this mainly means that we need to calculate the weights for the linear combination
        // given that a specific doorway may have been reached through more than one path in the graph, this is not a trivial task
        // we are going to do an exhaustive graph traversal, starting at the source node,
        // and we'll keep adding weight to the doorway based on how much gas reaches it through the specific path we are considering
        struct NodeState
        {
            float remainingGas;
            const DoorwayNode* doorSource;
            std::stack<const DoorwayNode*> doorways;
        };
        std::queue<NodeState> stateStack;

        SimWithResult result = SimulateSingleRoomFromAABB(sourceNode, sourceNode->GetAABB(), {});
        completeGasMap.gasMaps[sourceNode] = *result.hitMap;
        for (size_t i = 0; i < sourceNode->doorways.size(); i++)
        {
            const DoorwayNode& doorway = sourceNode->doorways.at(i);

            if (!Is<RoomNode>(doorway.to))
                continue;

            NodeState nextRoom;
            nextRoom.doorSource = &doorway.OtherSide();
            nextRoom.remainingGas = result.ProportionInDoorway(i);

            GSL_INFO("{}->{}   -   {}", doorway.from.lock()->id, doorway.to.lock()->id, nextRoom.remainingGas);

            for (const auto& nextDoorway : doorway.to.lock()->doorways)
                if (&nextDoorway != nextRoom.doorSource)
                    nextRoom.doorways.push(&nextDoorway);

            totalGasThroughDoorway[nextRoom.doorSource] = nextRoom.remainingGas;
            stateStack.push(nextRoom);
        }

        constexpr float minimumGasThr = 1e-1;
        while (!stateStack.empty())
        {
            NodeState& current = stateStack.front();

            // if we cannot keep expanding this node, pop it from the stack
            if (current.remainingGas < minimumGasThr || current.doorways.empty())
                stateStack.pop();
            else
            {
                // otherwise, let's get the next doorway and continue
                NodeState next;
                next.doorSource = &current.doorways.top()->OtherSide();

                // if no gas exits this room at all (a dead end or other weird edge case), just stop expansion in this direction
                SimWithResult result;
                if(simulationCache.contains(current.doorSource))
                    result = simulationCache.at(current.doorSource);
                else
                    result = SimulateSingleRoomFromDoorway(*current.doorSource);

                std::shared_ptr<Simulation> simulation = result.simulation;
                if (simulation->outlets->totalExitCount == 0)
                {
                    stateStack.pop();
                    continue;
                }

                GSL_INFO("{}->{}", next.doorSource->to.lock()->id, next.doorSource->from.lock()->id);
                // calculate how much of the gas in the current node makes it to the next node
                size_t outletIndex = next.doorSource->OtherSide().GetIndex();
                float gasProportion = result.ProportionInDoorway(outletIndex);

                GSL_INFO("Prop: {}", gasProportion);
                next.remainingGas = current.remainingGas * gasProportion;

                // update the total amount of gas that passes through the doorway
                totalGasThroughDoorway[next.doorSource] += next.remainingGas;

                // fill in the doorways of the next state node
                for (const auto& nextDoorway : next.doorSource->from.lock()->doorways)
                    if (&nextDoorway != next.doorSource)
                        next.doorways.push(&nextDoorway);

                // update the current state
                current.doorways.pop();
                current.remainingGas -= next.remainingGas;

                // push the new state on top
                stateStack.push(next);
            }
        }

        // OK, now we've done all that, we can combine the individual simulation maps,
        // weighted by the amount of gas that should have passed through each doorway

        for (const auto& node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room || node == sourceNode)
                continue;

            completeGasMap.gasMaps[room] = std::vector<float>(room->GetOccupancy().data.size(), 0.);
            for (const auto& doorway : node->doorways)
            {
                if (!totalGasThroughDoorway.contains(&doorway))
                    continue;

                if (!simulationCache.contains(&doorway))
                {
                    GSL_ASSERT(totalGasThroughDoorway.at(&doorway) < minimumGasThr);
                    continue;
                }

                float weight = totalGasThroughDoorway.at(&doorway);
                const auto& localHitmap = simulationCache.at(&doorway).hitMap;
                for (size_t i = 0; i < localHitmap->size(); i++)
                    completeGasMap.gasMaps[room].at(i) += localHitmap->at(i) * weight;
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

    MarkerArray SimulationSystem::VisualizeCachedResults(std::shared_ptr<PlaceNode> sourceRoom, float nodeSeparationViz)
    {
        MarkerArray array;
        CompleteMap& map = gasMapsWithRoomSource[sourceRoom];
        size_t i = 0;
        for (const auto& [room, result] : map.gasMaps)
        {
            std::vector<ColorRGBA> colors(result.size());
            for (size_t i = 0; i < result.size(); i++)
                colors.at(i) = Utils::valueToColor(result.at(i), 0, 1, Utils::ValueColorMode::Linear);

            Grid2D<Occupancy> occupancy = room->GetOccupancy();
            Grid2DMetadata vizMetadata = occupancy.metadata;
            vizMetadata.origin = vizMetadata.origin * nodeSeparationViz;

            Marker marker = Utils::createPointsMarker(Grid2D<ColorRGBA>(colors, occupancy.occupancy, vizMetadata));
            marker.id = i++;
            array.markers.push_back(marker);
        }
        return array;
    }

    float SimulationSystem::SimWithResult::NACatOutlet(size_t index)
    {
        const auto& mask = simulation->outlets->mask;

        float sum = 0;
        for (size_t i = 0; i < mask.data.size(); i++)
            if (mask.occupancy.at(i) && mask.data.at(i) == index)
                sum += hitMap->at(i);

        return sum / simulation->outlets->numCellsOutlet.at(index);
    }

    float SimulationSystem::SimWithResult::ProportionInDoorway(size_t index)
    {
        // return std::clamp((float)simulation->outlets->exitsPerOutlet.at(index) / maxBeforeNormalize, 0.f, 1.f);
        return NACatOutlet(index);
    }

} // namespace GSL::Graph_internal