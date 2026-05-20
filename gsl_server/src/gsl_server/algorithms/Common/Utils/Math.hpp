#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include <cfloat>
#include <gsl_server/algorithms/Common/Occupancy.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <vector>

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

    template <typename T>
    void NormalizeDistribution(std::vector<T>& variable, std::function<double&(T&)> accessor, const std::vector<Occupancy>& occupancy);
    template <typename T>
    void NormalizeDistribution(std::vector<T>& variable, const std::vector<Occupancy>& occupancy);

    void LogMaxNormalize(std::vector<float>& vec, const std::vector<Occupancy>& occupancy);
    void PowerMaxNormalize(std::vector<float>& vec, const std::vector<Occupancy>& occupancy, float power = 1);
    void Winsorize(std::vector<float>& vec, float percentile = 5);

    float EquallyDistributed01F();

    Vector2 Mode(const Grid2D<double> grid);

    Vector2 ExpectedValue(const Grid2D<double> grid, double proportionBest);
    double Variance(const Grid2D<double> grid);
    struct CovarianceMatrix
    {
        float x;
        float y;
        float covariance;
    };
    CovarianceMatrix Covariance(Grid2D<double> grid);

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
        double weight_sum = 0;
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
//--------------
template <typename CollectionIterator>
inline float GSL::Utils::getAverageFloatCollection(const CollectionIterator startIt, const CollectionIterator endIt)
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
inline float GSL::Utils::getAverageDirection(const CollectionIterator startIt, const CollectionIterator endIt)
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
inline double GSL::Utils::KLD(
    const std::vector<T>& a,
    const std::vector<T>& b,
    const std::vector<Occupancy>& occupancy,
    std::function<double(const T&)> accessor)
{
    double total = 0;
    for (int index = 0; index < a.size(); index++)
        if (occupancy[index] == Occupancy::Free)
        {
            double aVal = accessor(a[index]);
            double bVal = accessor(b[index]);
            double aux = aVal * std::log(aVal / bVal) + (1 - aVal) * std::log((1 - aVal) / (1 - bVal));
            total += std::isnan(aux) ? 0 : aux;
        }
    return total;
}

template <typename T>
void GSL::Utils::NormalizeDistribution(std::vector<T>& variable, const std::vector<GSL::Occupancy>& occupancy)
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

template <typename T>
inline void GSL::Utils::NormalizeDistribution(std::vector<T>& variable, std::function<double&(T&)> accessor, const std::vector<Occupancy>& occupancy)
{
    double total = 0;
    for (int i = 0; i < variable.size(); i++)
    {
        if (occupancy[i] == Occupancy::Free)
            total += accessor(variable[i]);
    }

#pragma omp parallel for
    for (int i = 0; i < variable.size(); i++)
    {
        if (occupancy[i] == Occupancy::Free)
            accessor(variable[i]) = accessor(variable[i]) / total;
    }
}
