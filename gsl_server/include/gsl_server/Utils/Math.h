#pragma once
#include <vector>
#include <deque>
#include <gsl_server/core/Vectors.h>

namespace GSL::Utils
{
    float get_average_vector(std::vector<float> const& v);
    float get_average_deque(std::deque<float> const& v);

    double lerp(double start, double end, double proportion);
    double remapRange(double value, double low1, double high1, double low2, double high2);
    double clamp(double val, double min, double max);

    double evaluate1DGaussian(double distance, double sigma);
    double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation);
    double logOddsToProbability(double l);

    double randomFromGaussian(double mean, double stdev);
    double uniformRandom(double min, double max);

    double KLD(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b);

} // namespace GSL::Utils