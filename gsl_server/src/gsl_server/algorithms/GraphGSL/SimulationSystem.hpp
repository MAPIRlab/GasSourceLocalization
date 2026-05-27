#pragma once

#include "Simulations.hpp"
#include "gsl_server/algorithms/Common/Simulation.hpp"
#include "gsl_server/algorithms/GraphGSL/Graph.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"
#include <stack>

namespace GSL::Graph_internal
{
    class SimulationSystem
    {
    public:
        void blurTest(std::vector<Vector2Int> points);
        SimulationSystem() : simulationCache(this) {}
        void Reset(); // remove all the cached data and results, get ready to run new simulations

        SimWithResult SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> node, Vector2 point);
        SimWithResult SimulateSingleRoomFromAABB(const std::shared_ptr<RoomNode> roomNode, AABB2D sourceAABB,
                                                 const std::set<std::shared_ptr<DoorwayNode>>& blockedDoorways);
        SimWithResult SimulateSingleRoomFromDoorway(const std::shared_ptr<const DoorwayNode> doorway);
        CompleteMap& SimulateEntireGraph(const std::shared_ptr<PlaceNode> firstNodeInSim, Vector2 sourcePoint);
        CompleteMap& SimulateEntireGraph(std::shared_ptr<DoorwayNode> sourceDoorway);
        
        MarkerArray VisualizeCachedResults(std::shared_ptr<PlaceNode> sourceRoom, size_t simulationIndex, float nodeSeparationViz);
        
        
        std::map<std::shared_ptr<PlaceNode>, std::deque<CompleteMap>> gasMapsWithRoomSource;
        Graph* graph;
        Options options;

        // the simulation cache is further encapsulated into its own class
        class SimulationCache
        {
        public:
            SimulationCache(SimulationSystem* simSys) : simSys(simSys) {}
            SimWithResult Get(std::shared_ptr<const DoorwayNode> doorway);
            void Clear();
            bool Contains(const std::shared_ptr<const DoorwayNode> doorway) { return simulations.contains(doorway); }
            bool IsRunning(const std::shared_ptr<const DoorwayNode> doorway);

        private:
            template <typename T, typename U>
            bool SyncContains(const T&, const U&);

            std::map<std::shared_ptr<const DoorwayNode>, SimWithResult> simulations;
            std::set<std::shared_ptr<const DoorwayNode>> simsInFlight;
            std::mutex mtx;
            SimulationSystem* simSys;
        } simulationCache;

    private:
        struct NodeState
        {
            float gasAtInlet;
            std::shared_ptr<const DoorwayNode> doorSource;
            std::stack<std::shared_ptr<const DoorwayNode>> doorways;
        };

        std::mutex mtx;
        std::map<std::shared_ptr<RoomNode>, std::optional<SimulationBlurMask>> blurMasks;

        void _SimulateEntireGraph(const std::shared_ptr<PlaceNode> firstNodeInSim, CompleteMap& completeMap);
        void PropagateSimThroughGraph(std::deque<NodeState>& stateStack, CompleteMap& completeGasMap, const std::shared_ptr<PlaceNode> firstNodeInSim);
    };

} // namespace GSL::Graph_internal