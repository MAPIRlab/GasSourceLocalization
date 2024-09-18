#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <xxHash/xxhash32.h>
#include <random>
#include <chrono>

namespace GSL::Utils
{
    static thread_local std::minstd_rand0 RNGengine;

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
            dist = std::normal_distribution<> {0, stdev};
            previousStdev = stdev;
        }

        return mean + dist(RNGengine);
    }

    double uniformRandom(double min, double max)
    {
#if 1
        // xxHash-based RNG. I don't really know much about it, but it goes vroom vroom
        static thread_local uint32_t seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        static thread_local uint32_t state = 0xFFFF;
        state = XXHash32::hash(&state, sizeof(state), seed);

        constexpr double reciprocalMax = 1 / (double)std::numeric_limits<uint32_t>::max();
        double rndVal01 = state * reciprocalMax;
        return min + rndVal01 * (max - min);
#else
        static thread_local std::uniform_real_distribution<double> distribution {0.0, 0.999};
        return min + distribution(RNGengine) * (max - min);
#endif
    }

    double KLD(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b)
    {
        double total = 0;
        for (int r = 0; r < a.size(); r++)
        {
            for (int c = 0; c < a[0].size(); c++)
            {
                double aux = a[r][c] * std::log(a[r][c] / b[r][c]) + (1 - a[r][c]) * std::log((1 - a[r][c]) / (1 - b[r][c]));
                total += std::isnan(aux) ? 0 : aux;
            }
        }
        return total;
    }

    void NormalizeDistribution(Grid2D<double> variable)
    {
        // we account for the possibility of having positive and negative values by offsetting everything by the value of the minimum (capped at 0)
        // so, [-1, -0.5, 1, 2] would become [0, 0.5, 2, 3] before the normalization happens
        double total = 0;
        int count = 0;
        for (int i = 0; i < variable.data.size(); i++)
        {
            if (variable.occupancy[i] == Occupancy::Free)
            {
                total += variable.data[i];
                count++;
            }
        }

        #pragma omp parallel for
        for (int i = 0; i < variable.data.size(); i++)
        {
            if (variable.occupancy[i] == Occupancy::Free)
            {
                variable.data[i] = variable.data[i] / total;
            }
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