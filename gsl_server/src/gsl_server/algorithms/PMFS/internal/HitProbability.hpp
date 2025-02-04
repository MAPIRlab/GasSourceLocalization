#pragma once
#include "Dirichlet.hpp"
#include "gsl_server/core/Logging.hpp"
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <numeric>
#include <optional>

namespace GSL::PMFS_internal
{
    struct HitProbability
    {
        Vector2 originalPropagationDirection;
        float distanceFromRobot = -1;
        float previousInfluence = -1;

        static constexpr size_t numBuckets = 10;
        std::array<float, numBuckets> alphas{};

        void addFrequencyEvidence(float freq, float mass)
        {
            size_t index = freqToIndex(freq);
            alphas[index] += mass;
            GSL_ASSERT(std::reduce(alphas.begin(), alphas.end(), 0) >= mass);

            update(); // distribution has changed, discard cached metrics
        }

        float probability(float freq)
        {
            float sum = std::reduce(alphas.begin(), alphas.end(), 0);
            size_t index = freqToIndex(freq);
            return alphas[index] / sum;
        }

        float entropy()
        {
            if (!_entropy)
                _entropy = Dirichlet::expectedShannonEntropy(alphas);
            GSL_ASSERT(*_entropy >= 0);
            return *_entropy;
        }

        float expectedValue()
        {
            if (!_expectedValue)
            {
                if (!probabilities)
                    calculateProbabilities();
                calculateExpectedValue();
            }
            return *_expectedValue;
        }

        // New, experimental idea:
        // don't work with p(H_i), but with p(f_i) -- which is essentially p(p(H_i))
        std::array<float, numBuckets> frequencyDistribution()
        {
            if (!probabilities)
                calculateProbabilities();

            return *probabilities;
        }

        static float frequencyOfBucket(uint index)
        {
            return (0.5f + index) / numBuckets;
        }

    private:
        void update()
        {
            _entropy = Dirichlet::expectedShannonEntropy(alphas);
            calculateProbabilities();
            calculateExpectedValue();
        }

        void calculateProbabilities()
        {
            probabilities.emplace();
            float sum = std::reduce(alphas.begin(), alphas.end(), 0);
            for (size_t i = 0; i < numBuckets; i++)
                (*probabilities)[i] = alphas[i] / sum;
        }

        void calculateExpectedValue()
        {
            _expectedValue = 0;
            for (size_t i = 0; i < probabilities->size(); i++)
                _expectedValue.value() += (*probabilities)[i] * frequencyOfBucket(i);
        }

        size_t freqToIndex(float freq){return freq * (numBuckets -1 );}

        std::optional<float> _entropy;
        std::optional<float> _expectedValue;
        std::optional<std::array<float, numBuckets>> probabilities;
    };

} // namespace GSL::PMFS_internal