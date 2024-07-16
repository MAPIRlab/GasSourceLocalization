#pragma once
#include <vector>
#include <deque>
#include <gsl_server/core/Vectors.hpp>
#include <cfloat>
#include <gsl_server/algorithms/Common/Grid.hpp>

namespace GSL::Utils
{
    static constexpr float Deg2Rad = M_PI / 180.0f;
    static constexpr float Rad2Deg = 180.0f / M_PI;

	static constexpr float INVALID_AVERAGE = -FLT_MAX;
	
	template <typename CollectionIterator>
    float getAverageFloatCollection(const CollectionIterator startIt, const CollectionIterator endIt)
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
	float getAverageDirection(const CollectionIterator startIt, const CollectionIterator endIt)
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

    double lerp(double start, double end, double proportion);
    double remapRange(double value, double low1, double high1, double low2, double high2);
    double clamp(double val, double min, double max);

    double evaluate1DGaussian(double distance, double sigma);
    double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation);
    double logOddsToProbability(double l);

    double randomFromGaussian(double mean, double stdev);
    double uniformRandom(double min, double max);

    double KLD(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b);
    void NormalizeDistribution(Grid<double>& variable);
    
    float EquallyDistributed01F();

    //holds a long list of N(0,1) values, and returns them one at a time, scaled as requested.
    //obviously not as good as generating them on the fly, but it's not like we are doing cryptography here
    template<int Size>
    class PrecalculatedGaussian
    {
    public:
        PrecalculatedGaussian()
        {
            m_index = uniformRandom(0, Size);
            for(size_t i = 0; i<Size; i++)
                m_precalculatedTable[i] = Utils::randomFromGaussian(0, 1);
        }

        float nextValue(float mean, float stdev)
        {
            m_index = (m_index+1) % Size;
            return mean + stdev*m_precalculatedTable[m_index];
        }
    private:
        uint16_t m_index;
        std::array<float, Size> m_precalculatedTable;
    };

} // namespace GSL::Utils