#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include <cfloat>
#include <gsl_server/algorithms/Common/Occupancy.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <vector>

template <typename T>
concept NumericType = std::integral<T> || std::floating_point<T>;

namespace GSL::Utils
{
    static constexpr float Deg2Rad = M_PI / 180.0f;
    static constexpr float Rad2Deg = 180.0f / M_PI;

    static constexpr float INVALID_AVERAGE = -FLT_MAX;

    template <typename CollectionIterator>
    float getAverageFloatCollection(const CollectionIterator startIt, const CollectionIterator endIt);

    template <typename CollectionIterator>
    float getAverageDirection(const CollectionIterator startIt, const CollectionIterator endIt);

    bool approx(double v1, double v2, double epsilon = 1e-5);

    double lerp(double start, double end, double proportion);
    double remapRange(double value, double low1, double high1, double low2, double high2);
    double clamp(double val, double min, double max);

    double evaluate1DGaussian(double distance, double sigma);
    double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation);
    double logOddsToProbability(double l);

    double randomFromGaussian(double mean, double stdev);
    double uniformRandom(double min, double max);
    float uniformRandomF(float min, float max);
    Vector2 polarToCartesian(float r, float theta);

    template <typename T>
    double KLD(
        const std::vector<T>& a,
        const std::vector<T>& b,
        const std::vector<Occupancy>& occupancy,
        std::function<double(const T&)> accessor);

    template <std::floating_point T>
    void NormalizeDistribution(std::vector<T>& variable, std::function<double&(T&)> accessor, const std::vector<Occupancy>& occupancy);
    template <std::floating_point T>
    void NormalizeDistribution(std::vector<T>& variable, const std::vector<Occupancy>& occupancy);

    void LogMaxNormalize(std::vector<float>& vec, const std::vector<Occupancy>& occupancy);
    template <std::floating_point T>
    void PowerMaxNormalize(std::vector<T>& vec, const std::vector<Occupancy>& occupancy, T power = 1);
    void Winsorize(std::vector<float>& vec, float percentile = 5);

    float EquallyDistributed01F();

    Vector2 Mode(const Grid2D<double> grid);

    template <std::floating_point T>
    Vector2 ExpectedValue(const MultiGrid<T> grid, double proportionBest = 1.0);

    // technically, *generalized* variance, defined as the trace of the covariance matrix
    template <std::floating_point T>
    double Variance(const MultiGrid<T> grid);

    struct CovarianceMatrix
    {
        float x;
        float y;
        float covariance;
    };
    template <std::floating_point T>
    CovarianceMatrix Covariance(MultiGrid<T> grid);

    // holds a long list of N(0,1) values, and returns them one at a time, scaled as requested.
    // obviously not as good as generating them on the fly, but it's not like we are doing cryptography here
    template <int Size>
    class PrecalculatedGaussian
    {
    public:
        PrecalculatedGaussian()
        {
            m_index = uniformRandom(0, Size);
            for (size_t i = 0; i < Size; i++)
                m_precalculatedTable[i] = Utils::randomFromGaussian(0, 1);
        }

        float nextValue(float mean, float stdev)
        {
            m_index = (m_index + 1) % Size;
            return mean + stdev * m_precalculatedTable[m_index];
        }

    private:
        uint16_t m_index;
        std::array<float, Size> m_precalculatedTable;
    };

    // See Updating Mean and Variance Estimates: An Improved Method D.H.D. West 1979
    struct RunningVariance
    {
        double mean = 0;
        double weight_sum = 1e-12; // avoid NaNs if there are several 0-weight values
        double weight_squared_sum = 0;
        double variance = 0;

        void Update(float value, float weight)
        {
            weight_sum = weight_sum + weight;
            weight_squared_sum = weight_squared_sum + weight * weight;
            double mean_old = mean;
            mean = mean_old + (weight / weight_sum) * (value - mean_old);
            variance = variance + weight * (value - mean_old) * (value - mean);
        }
    };

} // namespace GSL::Utils

// Template Definitions
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
namespace GSL::Utils
{

    template <typename CollectionIterator>
    inline float getAverageFloatCollection(const CollectionIterator startIt, const CollectionIterator endIt)
    {
        int length = std::distance(startIt, endIt);
        if (length == 0)
            return Utils::INVALID_AVERAGE;
        float sum = 0.0;
        for (CollectionIterator i = startIt; i != endIt; ++i)
            sum += *i;

        return sum / length;
    }

    template <typename CollectionIterator>
    inline float getAverageDirection(const CollectionIterator startIt, const CollectionIterator endIt)
    {
        // Average of wind direction, avoiding the problems of +/- pi angles.
        int length = std::distance(startIt, endIt);
        if (length == 0)
            return Utils::INVALID_AVERAGE;

        float x = 0.0, y = 0.0;
        for (CollectionIterator i = startIt; i != endIt; ++i)
        {
            x += cos(*i);
            y += sin(*i);
        }
        float average_angle = atan2(y, x);

        return average_angle;
    }

    // Kullback-Leibler Divergence
    template <typename T>
    inline double KLD(
        const std::vector<T>& a,
        const std::vector<T>& b,
        const std::vector<Occupancy>& occupancy,
        std::function<double(const T&)> accessor)
    {
        double total = 0;
        for (int index = 0; index < a.size(); index++)
            if (occupancy[index])
            {
                double aVal = accessor(a[index]);
                double bVal = accessor(b[index]);
                double aux = aVal * std::log(aVal / bVal) + (1 - aVal) * std::log((1 - aVal) / (1 - bVal));
                total += std::isnan(aux) ? 0 : aux;
            }
        return total;
    }

    template <std::floating_point T>
    void NormalizeDistribution(std::vector<T>& variable, const std::vector<GSL::Occupancy>& occupancy)
    {
        T total = 0;
        for (int i = 0; i < variable.size(); i++)
        {
            if (occupancy[i] == GSL::Occupancy::Free)
                total += variable[i];
        }

#pragma omp parallel for
        for (int i = 0; i < variable.size(); i++)
        {
            if (occupancy[i] == GSL::Occupancy::Free)
                variable[i] = variable[i] / total;
        }
    }

    template <std::floating_point T>
    inline void NormalizeDistribution(std::vector<T>& variable, std::function<double&(T&)> accessor, const std::vector<Occupancy>& occupancy)
    {
        double total = 0;
        for (int i = 0; i < variable.size(); i++)
        {
            if (occupancy[i])
                total += accessor(variable[i]);
        }

#pragma omp parallel for
        for (int i = 0; i < variable.size(); i++)
        {
            if (occupancy[i])
                accessor(variable[i]) = accessor(variable[i]) / total;
        }
    }

    template <std::floating_point T>
    void PowerMaxNormalize(std::vector<T>& vec, const std::vector<Occupancy>& occupancy, T power)
    {
        T max = 0;
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (occupancy.at(i) != Occupancy::Free)
                continue;
            vec.at(i) = std::pow(vec.at(i), power);
            max = std::max(max, vec.at(i));
        }

        if (max == 0)
            return;

#pragma omp parallel for
        for (size_t i = 0; i < vec.size(); i++)
        {
            if (occupancy.at(i) != Occupancy::Free)
                continue;
            vec.at(i) = vec.at(i) / max;
        }
    }

    template <std::floating_point T>
    Vector2 ExpectedValue(MultiGrid<T> mgrid, double proportionBest)
    {
        struct CellData
        {
            Vector2 coords;
            double probability;
            CellData(Vector2 coord, double prob)
            {
                coords = coord;
                probability = prob;
            }
        };
        std::vector<CellData> cellData;
        for (auto it = mgrid.begin(); it != mgrid.end(); ++it)
        {
            auto [data, occupancy] = *it;
            Vector2 coords = it.currentMetadata().indexToCoordinates(it.cellIdx);
            if (occupancy)
            {
                CellData cd(coords, data);
                cellData.push_back(cd);
            }
        }

        std::sort(cellData.begin(), cellData.end(), [](const CellData& a, const CellData& b)
                  {
                      return a.probability > b.probability;
                  });

        double averageX = 0, averageY = 0;
        double sum = 0;

        for (int i = 0; i < cellData.size() * proportionBest; i++)
        {
            CellData& cd = cellData[i];
            averageX += cd.probability * cd.coords.x;
            averageY += cd.probability * cd.coords.y;
            sum += cd.probability;
        }
        return Vector2(averageX / sum, averageY / sum);
    }

    template <std::floating_point T>
    double Variance(const MultiGrid<T> mgrid)
    {
        Vector2 expected = ExpectedValue(mgrid, 1);
        double x = 0, y = 0;
        for (auto it = mgrid.begin(); it != mgrid.end(); ++it)
        {
            auto [data, occupancy] = *it;
            if (occupancy)
            {
                Vector2 coords = it.currentMetadata().indexToCoordinates(it.cellIdx);
                double p = data;
                x += pow(coords.x - expected.x, 2) * p;
                y += pow(coords.y - expected.y, 2) * p;
            }
        }
        return x + y;
    }

    template <std::floating_point T>
    CovarianceMatrix Covariance(MultiGrid<T> mgrid)
    {
        Vector2 expectedValue = ExpectedValue(mgrid);
        float varX = 0;
        float varY = 0;
        float covar = 0;
        for (auto it = mgrid.begin(); it != mgrid.end(); ++it)
        {
            auto [data, occupancy] = *it;
            if (occupancy)
            {
                Vector2 center = it.currentMetadata().indexToCoordinates(it.cellIdx);

                float xDiff = (center.x - expectedValue.x);
                float yDiff = (center.y - expectedValue.y);

                float prob = data;
                varX += prob * xDiff * xDiff;
                varY += prob * yDiff * yDiff;
                covar += prob * xDiff * yDiff;
            }
        }
        return {.x = varX, .y = varY, .covariance = covar};
    }
} // namespace GSL::Utils
