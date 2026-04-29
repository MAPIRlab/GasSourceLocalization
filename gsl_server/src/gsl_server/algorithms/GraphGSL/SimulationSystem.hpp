#pragma once

#include "gsl_server/algorithms/Common/Simulation.hpp"
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
        };

        // gas maps expected in each room, assuming a specific source location
        struct CompleteMap
        {
            std::map<std::shared_ptr<RoomNode>, std::vector<float>> gasMaps;
        };

    public:
        void Reset(); // remove all the cached data and results, get ready to run new simulations

        SimWithResult SimulateSingleRoomFromPoint(const std::shared_ptr<RoomNode> node, Vector2 point);
        SimWithResult SimulateSingleRoomFromDoorway(const DoorwayNode& arc);

        void SimulateEntireGraphFromRoom(const std::shared_ptr<RoomNode> node);

        struct Options
        {
            bool cummulativeMap = true;
            float blurSigma = 0.7;
            float noiseSTDev = 0.1;
            float warmupTimeAcc = 2.0;
            size_t iterationLimit = 300;
            size_t minWarmupIterations = 1000;
            size_t maxWarmupIterations = 2000;
            float normalizationPower = 1.0;
        };
        Options options;

        std::map<const DoorwayNode*, SimWithResult> simulationCache;
        std::map<std::shared_ptr<RoomNode>, CompleteMap> gasWithRoomSource;

    private:
        std::map<std::shared_ptr<RoomNode>, std::optional<SimulationBlurMask>> blurMasks;
    };
} // namespace GSL::Graph_internal