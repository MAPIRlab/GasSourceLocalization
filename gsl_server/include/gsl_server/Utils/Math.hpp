#pragma once
#include <vector>
#include <deque>
#include <gsl_server/core/Vectors.hpp>

namespace GSL::Utils
{
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
    

    //holds a long list of N(0,1) values, and returns them one at a time, scaled as requested.
    //obviously not as good as generating them on the fly, but it's not like we are doing cryptography here
    class PrecalculatedGaussian
    {
    public:
        PrecalculatedGaussian(size_t size);
        double nextValue(double mean, double stdev);
    private:
        std::vector<double> precalculatedTable;
    };

} // namespace GSL::Utils