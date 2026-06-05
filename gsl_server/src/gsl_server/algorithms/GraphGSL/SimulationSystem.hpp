#pragma once

#include "Simulations.hpp"
#include "gsl_server/algorithms/Common/Simulation.hpp"
#include "gsl_server/algorithms/GraphGSL/Graph.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"
#include <gsl_server/algorithms/Common/Utils/Synchronization.hpp>
#include <stack>

namespace GSL::Graph_internal
{
    class SimulationSystem
    {
    public:
        SimulationSystem() : simulationCache(this) {}
        void Reset(); // remove all the cached data and results, get ready to run new simulations

        SimWithResult SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> node, Vector2 point);
        SimWithResult SimulateSingleRoomFromAABB(const std::shared_ptr<RoomNode> roomNode, AABB2D sourceAABB,
                                                 const std::set<std::shared_ptr<DoorwayNode>>& blockedDoorways);
        SimWithResult SimulateSingleRoomFromDoorway(const std::shared_ptr<const DoorwayNode> doorway);
        CompleteMap& SimulateEntireGraph(const std::shared_ptr<PlaceNode> firstNodeInSim, Vector2 sourcePoint);
        CompleteMap& SimulateEntireGraph(std::shared_ptr<DoorwayNode> sourceDoorway);

        void EmergencyStop(); // to be called from the UI when there is an infinite loop
        
        MarkerArray VisualizeCachedResults(std::shared_ptr<PlaceNode> sourceRoom, size_t simulationIndex, float nodeSeparationViz);
        
        void blurTest(std::vector<Vector2Int> points); // this is a utility for testing the effects of the blur. Not part of the algorithm.
        
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
        struct GraphCacheEntry
        {
            bool complete = false;
            std::map<std::shared_ptr<const DoorwayNode>, float> gasProportion;
        };

        bool emergencyStopped = false;
        std::mutex mtx;
        std::map<std::shared_ptr<RoomNode>, std::optional<SimulationBlurMask>> blurMasks;
        
        struct DoorwayPair
        {
            std::shared_ptr<const DoorwayNode> start;
            std::shared_ptr<const DoorwayNode> end;

            friend bool operator<(const DoorwayPair& a, const DoorwayPair& b)
            {
                if (a.start != b.start)
                    return a.start < b.start;
                return a.end < b.end;
            }
        };
        Utils::Synced<std::map<DoorwayPair, float>> doorwayPairs;

        void _SimulateEntireGraph(const std::shared_ptr<PlaceNode> firstNodeInSim, CompleteMap& completeMap);
        void PropagateSimThroughGraph(std::deque<NodeState>& stateStack, CompleteMap& completeGasMap, const std::shared_ptr<PlaceNode> firstNodeInSim);
    };

} // namespace GSL::Graph_internal