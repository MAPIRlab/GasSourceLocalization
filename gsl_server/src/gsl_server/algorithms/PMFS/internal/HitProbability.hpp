#pragma once
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/core/Vectors.hpp>

namespace GSL::PMFS_internal
{
    struct HitProbability
    {
        double logOdds = 0;
        double auxWeight = 0;
        Vector2 originalPropagationDirection;
        double omega = 0;      // intermediate step for the confidence value, goes from 0 to +infinity
        double confidence = 0; // 0-1
        double distanceFromRobot = 0;

        void setProbability(double probability)
        {
            logOdds = std::log(probability / (1 - probability));
        }

        double probability()
        {
            return Utils::logOddsToProbability(logOdds);
        }

        static constexpr size_t numBuckets = 5;
        std::array<double, numBuckets> frequencyDistribution()
        {
            // TODO make this not horrible

            std::array<double, numBuckets> probs;
            double probOfMode = Utils::lerp(1. / numBuckets, 1, confidence);
            double probOthers = (1 - probOfMode) / (numBuckets - 1);
            probs.fill(probOthers);

            double probabilityOfHit = probability();
            size_t indexOfMode = std::min<size_t>(probabilityOfHit * numBuckets, numBuckets - 1);
            probs[indexOfMode] = probOfMode;
            return probs;
        }
    };

} // namespace GSL::PMFS_internal