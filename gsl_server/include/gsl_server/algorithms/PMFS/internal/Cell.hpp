#pragma once
#include <gsl_server/core/Vectors.hpp>

namespace GSL::PMFS_internal
{
    struct Cell
    {
        Cell()
        {
            free = false;
            distanceFromRobot = 0;
            sourceProbability = 0;
            hitProbability.logOdds = 0;
            hitProbability.auxWeight = 0;
            hitProbability.omega = 0;
            hitProbability.confidence = 0;
        }
        bool free;
        double distanceFromRobot;
        struct HitProbability
        {
            double logOdds;
            double auxWeight;
            Vector2 originalPropagationDirection;
            double omega;      // intermediate step for the confidence value, goes from 0 to +infinity
            double confidence; // 0-1
        };
        HitProbability hitProbability;

        double sourceProbability;
    };
} // namespace GSL::PMFS