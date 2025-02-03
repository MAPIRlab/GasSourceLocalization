#pragma once
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <numeric>

namespace GSL::PMFS_internal
{
    struct HitProbability
    {
        Vector2 originalPropagationDirection;
        float distanceFromRobot = -1;
        float previousInfluence = -1;

        static constexpr size_t numBuckets = 5;
        std::array<float, numBuckets> alphas{};

        void addFrequencyEvidence(float freq, float mass)
        {
            size_t index = numBuckets / freq;
            alphas[index] += mass;
        }

        float probability(float freq)
        {
            float sum = std::reduce(alphas.begin(), alphas.end(), 0);
            size_t index = numBuckets / freq;
            return alphas[index] / sum;
        }

        // New, experimental idea:
        // don't work with p(H_i), but with p(f_i) -- which is essentially p(p(H_i))
        std::array<float, numBuckets> frequencyDistribution()
        {
            std::array<float, numBuckets> probabilities;
            float sum = std::reduce(alphas.begin(), alphas.end(), 0);
            for (size_t i = 0; i < numBuckets; i++)
                probabilities[i] = alphas[i] / sum;
            return probabilities;
        }

        static float frequencyOfBucket(uint index)
        {
            return (0.5f + index) / numBuckets;
        }
    };

} // namespace GSL::PMFS_internal