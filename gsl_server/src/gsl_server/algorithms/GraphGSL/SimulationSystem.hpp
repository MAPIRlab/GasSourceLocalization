#pragma once

#include "Simulations.hpp"
#include "gsl_server/algorithms/Common/Simulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Synchronization.hpp"
#include "gsl_server/algorithms/GraphGSL/Graph.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"

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
        MarkerArray VisualizeCachedResults(std::shared_ptr<PlaceNode> sourceRoom, size_t simulationIndex, float nodeSeparationViz);
        void SimulateEntireGraph(const std::shared_ptr<PlaceNode> sourceNode, Vector2 sourcePoint);

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
        std::mutex mtx;
        std::map<std::shared_ptr<RoomNode>, std::optional<SimulationBlurMask>> blurMasks;

        void _SimulateEntireGraph(const std::shared_ptr<PlaceNode> sourceNode, CompleteMap& completeMap);

        struct GraphCacheEntry
        {
            bool complete = false;
            std::map<std::shared_ptr<const DoorwayNode>, float> gasProportion;
        };
        Utils::Synced<std::map<std::shared_ptr<const DoorwayNode>, GraphCacheEntry>> graphCache;
    };

} // namespace GSL::Graph_internal