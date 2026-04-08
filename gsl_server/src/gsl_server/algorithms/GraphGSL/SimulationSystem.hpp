#pragma once

#include "gsl_server/algorithms/Common/Simulation.hpp"
#include <vector>

namespace GSL
{
    class Arc;
    class RealNode;
} // namespace GSL

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

    public:
        static SimWithResult SimulateFromPoint(const std::shared_ptr<RealNode> node, Vector2 point);
        static SimWithResult SimulateFromArc(const Arc& arc);

        struct Options
        {
            bool cummulativeMap = true;
            float blurSigma = 0.7;
            float noiseSTDev = 0.1;
            size_t minWarmupIterations = 500;
            size_t maxWarmupIterations = 2000;
            float normalizationPower = 1.0;
        };
        static Options options;

    private:
        static inline std::map<std::shared_ptr<RealNode>, std::optional<SimulationBlurMask>> blurMasks;
    };
} // namespace GSL::Graph_internal