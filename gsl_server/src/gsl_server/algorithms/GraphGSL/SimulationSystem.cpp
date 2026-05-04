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

    SimulationSystem::SimWithResult SimulationSystem::SimulateSingleRoomFromDoorway(const DoorwayNode& doorway)
    {
        SimWithResult result;
        auto roomNode = As<RoomNode>(doorway.from.lock());
        GSL_ASSERT_MSG(roomNode, "Tried to do simulation in place node which is not a room: {}", doorway.from.lock()->id);
        Grid2DMetadata nodeMetadata = roomNode->GetOccupancy().metadata;
        AABB2D sourceAABB(doorway.aabb.min,
                          doorway.aabb.max);
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
            if (&roomNode->doorways.at(i) == &doorway)
                result.simulation->outlets->enabled.at(i) = false;

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        if (options.cummulativeMap)
        {
            Utils::Windsorize(*result.hitMap, 5);
            Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, options.normalizationPower);
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);
            Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1);
        }
        else
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);

        // store the simulation result in the cache
        //-------------------
        simulationCache[&doorway] = result;

        return result;
    }

    void SimulationSystem::SimulateEntireGraphFromRoom(const Graph& graph, const std::shared_ptr<RoomNode> sourceNode)
    {
        std::queue<const DoorwayNode*> simQueue;

        // add the doorways that exit the source room as a starting point
        for (const auto& doorway : sourceNode->doorways)
            simQueue.push(&doorway);

        // follow the gas from doorway to doorway, caching the simulations
        while (!simQueue.empty())
        {
            const DoorwayNode* doorway = simQueue.front();
            simQueue.pop();

            // if this simulation is already cached, skip
            if (simulationCache.contains(&doorway->OtherSide()))
                continue;

            if (!Is<RoomNode>(doorway->to))
                continue;

            GSL_INFO("Simulating {}->{}", doorway->from.lock()->id, doorway->to.lock()->id);
            const DoorwayNode* otherSide = &doorway->OtherSide();
            SimWithResult result = SimulateSingleRoomFromDoorway(*otherSide);

            const auto& doorways = doorway->to.lock()->doorways;
            for (size_t i = 0; i < doorways.size(); i++)
            {
                // only simulate next room if there is gas entering it
                if (simulationCache.at(otherSide).simulation->outlets->exitsPerOutlet.at(i) == 0)
                    continue;

                const DoorwayNode* nextDoorway = &doorways.at(i);
                simQueue.push(nextDoorway);
            }
        }

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
        std::stack<NodeState> stateStack; // stack makes the traversal depth-first (but that is ultimately arbitrary)

        // TODO do we simulate within the starting room? currently assuming all doorways are equally important
        for (const auto& doorway : sourceNode->doorways)
        {
            if (!Is<RoomNode>(doorway.to))
                continue;
            
            NodeState nextRoom;
            nextRoom.doorSource = &doorway.OtherSide();
            nextRoom.remainingGas = 1.f / sourceNode->doorways.size();
            for (const auto& nextDoorway : doorway.to.lock()->doorways)
                nextRoom.doorways.push(&nextDoorway);

            GSL_ASSERT(simulationCache.contains(nextRoom.doorSource));
            totalGasThroughDoorway[nextRoom.doorSource] = nextRoom.remainingGas;
            stateStack.push(nextRoom);
        }

        constexpr float minimumGasThr = 1e-2;
        while (!stateStack.empty())
        {
            NodeState& current = stateStack.top();

            // if we cannot keep expanding this node, pop it from the stack
            if (current.remainingGas < minimumGasThr || current.doorways.empty())
                stateStack.pop();
            else
            {
                // otherwise, let's get the next doorway and continue
                NodeState next;
                next.doorSource = &current.doorways.top()->OtherSide();

                // if no gas exits this room at all (a dead end or other weird edge case), just stop expansion in this direction
                std::shared_ptr<Simulation> simulation = simulationCache.at(current.doorSource).simulation;
                if (simulation->outlets->totalExitCount == 0)
                {
                    stateStack.pop();
                    continue;
                }

                // calculate how much of the gas in the current node makes it to the next node
                size_t outletIndex = next.doorSource->OtherSide().GetIndex();
                float gasProportion = (float)simulation->outlets->exitsPerOutlet.at(outletIndex) / simulation->outlets->totalExitCount;
                // float gasProportion = (float)simulation->outlets->exitsPerOutlet.at(outletIndex) / simulation->totalEmittedFilaments; //TODO is this better?

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

        CompleteMap& map = gasMapsWithRoomSource[sourceNode];
        for (const auto& node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;

            map.gasMaps[room] = std::vector<float>(room->GetOccupancy().data.size(), 0.);
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
                    map.gasMaps[room].at(i) += localHitmap->at(i) * weight;
            }
        }
    }

    MarkerArray SimulationSystem::VisualizeCachedResults(std::shared_ptr<RoomNode> sourceRoom)
    {
        MarkerArray array;
        CompleteMap& map = gasMapsWithRoomSource[sourceRoom];
        size_t i = 0;
        for (const auto& [room, result] : map.gasMaps)
        {
            std::vector<ColorRGBA> colors(result.size());
            for (size_t i = 0; i < result.size(); i++)
                colors.at(i) = Utils::valueToColor(result.at(i), 0, 1, Utils::ValueColorMode::Linear);
            Marker marker = Utils::createPointsMarker(Grid2D<ColorRGBA>(colors, room->GetOccupancy()));
            marker.id = i++;
            array.markers.push_back(marker);
        }
        return array;
    }

} // namespace GSL::Graph_internal