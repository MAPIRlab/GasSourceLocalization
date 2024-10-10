#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <random>
#include <chrono>
#include <xxHash/xxhash32.h>

namespace GSL::Utils
{
    static thread_local std::minstd_rand0 RNGengine;

    bool approx(double v1, double v2)
    {
        return std::abs(v1 - v2) < epsilon;
    }

    double lerp(double start, double end, double proportion)
    {
        if (proportion < 0 || std::isnan(proportion))
            return start;

        return start + (end - start) * std::min(1.0, proportion);
    }

    double clamp(double val, double min, double max)
    {
        return std::max(min, std::min(val, max));
    }

    double remapRange(double value, double low1, double high1, double low2, double high2)
    {
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    }

    double evaluate1DGaussian(double distance, double sigma)
    {
        return exp(-0.5 * (std::pow(distance, 2) / std::pow(sigma, 2))) / (sigma * std::sqrt(2 * M_PI));
    }

    double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation)
    {
        // we rotate the vector instead of the gaussian because keeping the distribution axis-aligned removes several terms from the PDF equation
        Vector2 v = vmath::rotate(sampleOffset, distributionRotation);

        return std::exp(-0.5 * (std::pow(v.x / sigma.x, 2) + std::pow(v.y / sigma.y, 2))) / (2 * M_PI * sigma.x * sigma.y);
    }

    double logOddsToProbability(double lo)
    {
        return 1.0 - 1.0 / (1 + std::exp(lo));
    }

    double randomFromGaussian(double mean, double stdev)
    {
        static thread_local std::normal_distribution<> dist{0, stdev};
        static thread_local double previousStdev = stdev;

        if (stdev != previousStdev)
        {
            dist = std::normal_distribution<>{0, stdev};
            previousStdev = stdev;
        }

        return mean + dist(RNGengine);
    }

    template <typename T> T uniformRandomT(T min, T max)
    {
#if 1
        // xxHash-based RNG. It's supposed to be faster 
        // !!!! the range is [0,1] inclusive!
        static thread_local uint32_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        static thread_local uint32_t state = 0xFFFF;
        state = XXHash32::hash(&state, sizeof(state), seed);
        state = std::min(state, std::numeric_limits<uint32_t>::max() - 1); // make it so the return interval does not include 1

        constexpr T reciprocalMax = 1. / (T)std::numeric_limits<uint32_t>::max();
        T rndVal01 = state * reciprocalMax;
        return min + rndVal01 * (max - min);
#else
        static thread_local std::uniform_real_distribution<T> distribution{0.0, 0.999};
        return min + distribution(RNGengine) * (max - min);
#endif
    }

    float uniformRandomF(float min, float max)
    {
        return uniformRandomT(min, max);
    }

    double uniformRandom(double min, double max)
    {
        return uniformRandomT(min, max);
    }

    void NormalizeDistribution(std::vector<double>& variable, std::vector<Occupancy>& occupancy)
    {
        double total = 0;
        for (int i = 0; i < variable.size(); i++)
        {
            if (occupancy[i] == Occupancy::Free)
                total += variable[i];
        }

#pragma omp parallel for
        for (int i = 0; i < variable.size(); i++)
        {
            if (occupancy[i] == Occupancy::Free)
                variable[i] = variable[i] / total;
        }
    }

    float EquallyDistributed01F()
    {
        constexpr float phi = 1.61803398875;
        static float current = 0;
        current += phi;
        return fmod(current, 1);
    }
} // namespace GSL::Utils