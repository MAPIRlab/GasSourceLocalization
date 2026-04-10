#include "gsl_server/core/Logging.hpp"
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <random>

namespace GSL::Utils
{
    static thread_local std::minstd_rand0 RNGengine;

    bool approx(double v1, double v2, double epsilon)
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
        static thread_local std::uniform_real_distribution<T> distribution{0.0, 0.999};
        return min + distribution(RNGengine) * (max - min);
    }

    float uniformRandomF(float min, float max)
    {
        return uniformRandomT(min, max);
    }

    Vector2 polarToCartesian(float r, float theta)
    {
        return r * Vector2(cos(theta), sin(theta));
    }

    double uniformRandom(double min, double max)
    {
        return uniformRandomT(min, max);
    }

    void PowerMaxNormalize(std::vector<float>& vec, const std::vector<Occupancy>& occupancy, float power)
    {
        float max = 0;
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (occupancy.at(i) != Occupancy::Free)
                continue;
            vec.at(i) = std::pow(vec.at(i), power);
            max = std::max(max, vec.at(i));
        }

#pragma omp parallel for
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (occupancy.at(i) != Occupancy::Free)
                continue;
            vec.at(i) = vec.at(i) / max;
        }
    }

    void Windsorize(std::vector<float>& vec, float percentile)
    {
        GSL_ASSERT(percentile > 0);
        GSL_ASSERT(percentile < 100);

        // take the value of the nth percentile to avoid outliers messing things up (unless it is 0, which means very few cells even contain any gas)
        std::vector<float> sorted;
        sorted.reserve(vec.size());
        std::copy(vec.begin(), vec.end(), std::back_inserter(sorted));
        std::sort(sorted.begin(), sorted.end());

        size_t index_nth = (100 - percentile) / 100. * sorted.size();
        float nth = sorted.at(index_nth);
        float maxVal = 0;

        if (nth > 0)
            maxVal = nth;
        else
        {
            // just take the first non-zero value
            for (size_t i = index_nth; i < sorted.size(); i++)
                if (sorted.at(i) > 0)
                    maxVal = sorted.at(i);
        }
        for (int i = 0; i < vec.size(); i++)
            vec[i] = std::clamp(vec[i], 0.f, maxVal);
    }

    void LogMaxNormalize(std::vector<float>& vec, const std::vector<Occupancy>& occupancy)
    {
        float max = 0;
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (occupancy.at(i) != Occupancy::Free)
                continue;
            vec.at(i) = std::log(vec.at(i) + 1);
            max = std::max(max, vec.at(i));
        }

#pragma omp parallel for
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (occupancy.at(i) != Occupancy::Free)
                continue;
            vec.at(i) = vec.at(i) / max;
        }
    }

    float EquallyDistributed01F()
    {
        constexpr float phi = 1.61803398875;
        static float current = 0;
        current += phi;
        return fmod(current, 1);
    }

    Vector2 Mode(const Grid2D<double> grid)
    {
        double best = 0;
        Vector2 position{0, 0};
        for (size_t i = 0; i < grid.data.size(); i++)
        {
            if (grid.data.at(i) > best)
            {
                best = grid.data.at(i);
                position = grid.metadata.indexToCoordinates(i);
            }
        }
        return position;
    }

    Vector2 ExpectedValue(const Grid2D<double> grid, double proportionBest)
    {
        struct CellData
        {
            Vector2Int indices;
            double probability;
            CellData(Vector2Int ind, double prob)
            {
                indices = ind;
                probability = prob;
            }
        };
        std::vector<CellData> data;
        for (int y = 0; y < grid.metadata.dimensions.y; y++)
        {
            for (int x = 0; x < grid.metadata.dimensions.x; x++)
            {
                if (grid.occupancy[grid.metadata.indexOf({x, y})] == Occupancy::Free)
                {
                    CellData cd(Vector2Int(x, y), grid.data[grid.metadata.indexOf({x, y})]);
                    data.push_back(cd);
                }
            }
        }

        std::sort(data.begin(), data.end(), [](const CellData& a, const CellData& b)
                  {
                      return a.probability > b.probability;
                  });

        double averageX = 0, averageY = 0;
        double sum = 0;

        for (int i = 0; i < data.size() * proportionBest; i++)
        {
            CellData& cd = data[i];
            Vector2 coord = grid.metadata.indicesToCoordinates(cd.indices.x, cd.indices.y);
            averageX += cd.probability * coord.x;
            averageY += cd.probability * coord.y;
            sum += cd.probability;
        }
        return Vector2(averageX / sum, averageY / sum);
    }

    double Variance(const Grid2D<double> grid)
    {
        Vector2 expected = ExpectedValue(grid, 1);
        double x = 0, y = 0;
        for (int row = 0; row < grid.metadata.dimensions.y; row++)
        {
            for (int col = 0; col < grid.metadata.dimensions.x; col++)
            {
                if (grid.occupancy[grid.metadata.indexOf({col, row})] == Occupancy::Free)
                {
                    Vector2 coords = grid.metadata.indicesToCoordinates(col, row);
                    double p = grid.data[grid.metadata.indexOf({col, row})];
                    x += pow(coords.x - expected.x, 2) * p;
                    y += pow(coords.y - expected.y, 2) * p;
                }
            }
        }
        return x + y;
    }

    CovarianceMatrix Covariance(Grid2D<double> grid)
    {
        Vector2 expectedValue = ExpectedValue(grid, 1);
        float varX = 0;
        float varY = 0;
        float covar = 0;
        for (int i = 0; i < grid.data.size(); i++)
        {
            if (grid.occupancy.at(i) == Occupancy::Free)
            {
                Vector2 center = grid.metadata.indexToCoordinates(i);

                float xDiff = (center.x - expectedValue.x);
                float yDiff = (center.y - expectedValue.y);

                float prob = grid.data.at(i);
                varX += prob * xDiff * xDiff;
                varY += prob * yDiff * yDiff;
                covar += prob * xDiff * yDiff;
            }
        }
        return {.x = varX, .y = varY, .covariance = covar};
    }
} // namespace GSL::Utils