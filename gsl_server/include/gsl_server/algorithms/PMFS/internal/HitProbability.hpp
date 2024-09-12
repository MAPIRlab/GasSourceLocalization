#pragma once
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/algorithms/Common/Grid.hpp>

namespace GSL::PMFS_internal
{
    struct HitProbability
    {
        double logOdds = 0;
        double auxWeight = 0;
        Vector2 originalPropagationDirection;
        double omega = 0; // intermediate step for the confidence value, goes from 0 to +infinity
        double confidence = 0; // 0-1
        double distanceFromRobot = 0;

        void setProbability(double probability)
        {
            logOdds = std::log( probability / (1 - probability) );
        }
    };

} // namespace GSL::PMFS