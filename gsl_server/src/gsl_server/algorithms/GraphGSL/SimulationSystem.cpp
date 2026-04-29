#include "SimulationSystem.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include <stack>

namespace GSL::Graph_internal
{
    void SimulationSystem::Reset()
    {
        simulationCache.clear();
        gasWithRoomSource.clear();
        // blurMasks.clear(); //this can probably be retained (if the graph does not change)
    }

    SimulationSystem::SimWithResult SimulationSystem::SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> realNode, Vector2 point)
    {
        ScopedStopwatch s("sims");
        SimWithResult result;
        result.hitMap = std::make_shared<std::vector<float>>(realNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(point),
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = realNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = realNode->GetOutletsMask(),
                .exitsPerOutlet = std::vector<size_t>(realNode->doorways.size(), 0),
            },
        });

        result.simulation->outlets->exitsPerOutlet.resize(realNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(realNode->doorways.size(), true);

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        Utils::Windsorize(*result.hitMap, 5);
        Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, options.normalizationPower);
        Simulation::blurHitMap(*result.hitMap, options.blurSigma, realNode->GetOccupancy(), blurMasks[realNode]);
        Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, 1);
        return result;
    }

    SimulationSystem::SimWithResult SimulationSystem::SimulateSingleRoomFromDoorway(const DoorwayNode& doorway)
    {
        SimWithResult result;
        auto realNode = As<RoomNode>(doorway.from.lock());
        Grid2DMetadata nodeMetadata = realNode->GetOccupancy().metadata;
        AABB2D sourceAABB(doorway.aabb.min,
                          doorway.aabb.max);
        Vector2 maxCoords = nodeMetadata.indicesToCoordinates(nodeMetadata.dimensions, false) - Vector2{0.001, 0.001};
        sourceAABB.min.x = std::clamp(sourceAABB.min.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.min.y = std::clamp(sourceAABB.min.y, nodeMetadata.origin.y, maxCoords.y);
        sourceAABB.max.x = std::clamp(sourceAABB.max.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.max.y = std::clamp(sourceAABB.max.y, nodeMetadata.origin.y, maxCoords.y);

        // configure the simulation
        result.hitMap = std::make_shared<std::vector<float>>(realNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(sourceAABB),
            .warmupAcceleration = options.warmupTimeAcc,
            .timesteps = options.iterationLimit,
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = realNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = realNode->GetOutletsMask(),
                .exitsPerOutlet = std::vector<size_t>(realNode->doorways.size(), 0),
                .numCellsOutlet = realNode->GetOutletsCellCount(),
            },
        });

        result.simulation->outlets->exitsPerOutlet.resize(realNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(realNode->doorways.size(), true);

        for (size_t i = 0; i < realNode->doorways.size(); i++)
            if (&realNode->doorways.at(i) == &doorway)
                result.simulation->outlets->enabled.at(i) = false;

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        if (options.cummulativeMap)
        {
            Utils::Windsorize(*result.hitMap, 5);
            Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, options.normalizationPower);
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, realNode->GetOccupancy(), blurMasks[realNode]);
            Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, 1);
        }
        else
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, realNode->GetOccupancy(), blurMasks[realNode]);

        // store the simulation result in the cache
        //-------------------
        simulationCache[&doorway] = result;

        return result;
    }

    void SimulationSystem::SimulateEntireGraphFromRoom(const std::shared_ptr<RoomNode> sourceNode)
    {
        std::queue<const DoorwayNode*> simQueue;

        for (const auto& doorway : sourceNode->doorways)
            if (!simulationCache.contains(&doorway))
                simQueue.push(&doorway);

        while (!simQueue.empty())
        {
            const DoorwayNode* doorway = simQueue.front();
            simQueue.pop();
            SimWithResult result = SimulateSingleRoomFromDoorway(*doorway);
            // TODO add the other doorways to the queue
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
            const DoorwayNode* from;
            std::stack<const DoorwayNode*> doorways;
        };
        std::stack<NodeState> stateStack; // stack makes the traversal depth-first (but that is ultimately arbitrary)

        // TODO do we simulate within the starting room? currently assuming all doorways are equally important
        for (const auto& doorway : sourceNode->doorways)
        {
            NodeState nextRoom;
            nextRoom.from = &doorway.OtherSide();
            nextRoom.remainingGas = 1.f / sourceNode->doorways.size();
            for (const auto& nextDoorway : doorway.to.lock()->doorways)
                nextRoom.doorways.push(&nextDoorway);

            totalGasThroughDoorway[&doorway] = nextRoom.remainingGas;
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
                next.from = &current.doorways.top()->OtherSide();

                // if no gas exits this room at all (a dead end or other weird edge case), just stop expansion in this direction
                std::shared_ptr<Simulation> simulation = simulationCache.at(current.from).simulation;
                if (simulation->outlets->totalExitCount == 0)
                    continue;

                // calculate how much of the gas in the current node makes it to the next node
                size_t outletIndex = next.from->OtherSide().GetIndex();
                float gasProportion = (float)simulation->outlets->exitsPerOutlet.at(outletIndex) / simulation->outlets->totalExitCount;
                // float gasProportion = (float)simulation->outlets->exitsPerOutlet.at(outletIndex) / simulation->totalEmittedFilaments; //TODO is this better?

                next.remainingGas = current.remainingGas * gasProportion;

                // update the total amount of gas that passes through the doorway
                totalGasThroughDoorway[next.from] += next.remainingGas;

                // fill in the doorways of the next state node
                for (const auto& nextDoorway : next.from->from.lock()->doorways)
                    if (&nextDoorway != next.from)
                        next.doorways.push(&nextDoorway);

                // update the current state
                current.doorways.pop();
                current.remainingGas -= next.remainingGas;

                // push the new state on top
                stateStack.push(next);
            }
        }
    }

} // namespace GSL::Graph_internal