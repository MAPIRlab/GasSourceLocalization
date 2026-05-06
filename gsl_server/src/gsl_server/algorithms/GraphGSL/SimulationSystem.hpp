#pragma once

#include "gsl_server/algorithms/Common/Simulation.hpp"
#include "gsl_server/algorithms/GraphGSL/Graph.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"
#include <vector>

namespace GSL::Graph_internal
{
    class SimulationSystem
    {
    public:
        struct SimWithResult
        {
            std::shared_ptr<Simulation> simulation;
            std::shared_ptr<std::vector<float>> hitMap;
            float maxBeforeNormalize = 0;

            float NACatOutlet(size_t index);
            float ProportionInDoorway(size_t index);
        };

        // gas maps expected in each room, assuming a specific source location
        struct CompleteMap
        {
            std::map<std::shared_ptr<RoomNode>, std::vector<float>> gasMaps;
        };

    public:
        void Reset(); // remove all the cached data and results, get ready to run new simulations

        SimWithResult SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> node, Vector2 point);
        SimulationSystem::SimWithResult SimulateSingleRoomFromAABB(const std::shared_ptr<RoomNode> roomNode, AABB2D sourceAABB,
                                                                   const std::set<const DoorwayNode*>& blockedDoorways);
        SimWithResult SimulateSingleRoomFromDoorway(const DoorwayNode& arc);

        void SimulateEntireGraphFromRoom(const Graph& graph, const std::shared_ptr<PlaceNode> node);

        MarkerArray VisualizeCachedResults(std::shared_ptr<PlaceNode> sourceRoom, float nodeSeparationViz);

        struct Options
        {
            bool cummulativeMap = true;
            float filamentsPerSecond = 2.0;
            float deltaTime = 0.2;
            float blurSigma = 3.0;
            float noiseSTDev = 0.1;
            float warmupTimeAcc = 8.0;
            size_t iterationLimit = 100;
            size_t minWarmupIterations = 500;
            size_t maxWarmupIterations = 1000;
            float normalizationPower = 1.0;
        };
        Options options;

        std::map<const DoorwayNode*, SimWithResult> simulationCache;
        std::map<std::shared_ptr<PlaceNode>, CompleteMap> gasMapsWithRoomSource;

        Graph* graph;

    private:
        std::map<std::shared_ptr<RoomNode>, std::optional<SimulationBlurMask>> blurMasks;
    };
} // namespace GSL::Graph_internal