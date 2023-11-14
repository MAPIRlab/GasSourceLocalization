#include <gsl_server/Utils/Math.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>
#include <random>

namespace GSL::Utils
{
    float getAverageVector(std::vector<float> const& v)
    {
        int length = v.size();
        float sum = 0.0;
        for (std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
            sum += *i;

        return sum / length;
    }

    float getAverageDeque(std::deque<float> const& v)
    {
        int length = v.size();
        float sum = 0.0;
        for (std::deque<float>::const_iterator i = v.begin(); i != v.end(); ++i)
            sum += *i;

        return sum / length;
    }

    float getAverageDirection(const std::vector<float>& vec)
    {
        // Average of wind direction, avoiding the problems of +/- pi angles.
        float x = 0.0, y = 0.0;
        for (std::vector<float>::const_iterator i = vec.begin(); i != vec.end(); ++i)
        {
            x += cos(*i);
            y += sin(*i);
        }
        float average_angle = atan2(y, x);

        return average_angle;
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
        Vector2 v = glm::rotate(sampleOffset, -distributionRotation);

        return std::exp(-0.5 * (std::pow(v.x / sigma.x, 2) + std::pow(v.y / sigma.y, 2))) / (2 * M_PI * sigma.x * sigma.y);
    }

    double logOddsToProbability(double lo)
    {
        return 1.0 - 1.0 / (1 + std::exp(lo));
    }

    double randomFromGaussian(double mean, double stdev)
    {
        static thread_local std::mt19937 engine;

        static thread_local std::normal_distribution<> dist{0, stdev};
        static thread_local double previousStdev = stdev;

        if (stdev != previousStdev)
        {
            dist = std::normal_distribution<>{0, stdev};
            previousStdev = stdev;
        }

        return mean + dist(engine);
    }

    double uniformRandom(double min, double max)
    {
        static thread_local std::mt19937 engine;
        static thread_local std::uniform_real_distribution<double> distribution{0.0, 0.999};

        return min + distribution(engine) * (max - min);
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

} // namespace GSL::Utils