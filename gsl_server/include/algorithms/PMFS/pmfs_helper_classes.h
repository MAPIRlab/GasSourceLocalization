#pragma once
#include <cmath>
#include <unordered_set>
#include <Utils/Utils.h>
#include <Utils/Time.h>
#include "spdlog/spdlog.h"
#include <NQA/NQAQuadtree.h>

namespace PMFS
{

    typedef Utils::Vector2Int Vector2Int;
    typedef std::unordered_set<Vector2Int, Vector2Int::Vec2IntHash, Vector2Int::Vec2IntCompare> hashSet;
    enum class State
    {
        WAITING_FOR_MAP,
        INITIALIZING,
        EXPLORATION,
        STOP_AND_MEASURE,
        MOVING
    };

    class Cell
    {
    public:
        Cell();
        ~Cell();
        bool free;
        double distanceFromRobot;
        struct hitProbability
        {
            double logOdds;
            double auxWeight;
            Utils::Vector2 originalPropagationDirection;
            double omega;      // intermediate step for the confidence value, goes from 0 to +infinity
            double confidence; // 0-1
        };
        hitProbability hitProbability;

        double sourceProbability;
    };

    struct HitProbKernel
    {
        double angle;
        Utils::Vector2 sigma;
        float valueAt1;
    };

    struct CellData
    {
        Vector2Int indices;
        double probability;
        CellData(Vector2Int ind, double prob)
        {
            indices = ind;
            probability = prob;
        }
        ~CellData() {}
    };

    struct SourceEstimationError
    {
        float error;
        float variance;
    };

    struct AveragedMeasurement
    {
        float wind_speed;
        float wind_direction;
        float concentration;

        static float get_average_wind_direction(std::vector<float> const& v);
    };
}