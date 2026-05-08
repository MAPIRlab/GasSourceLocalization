#include "NACCompare.hpp"
#include <eigen3/Eigen/Dense>

float LeastSquaresScale(const std::vector<float>& simulated, const std::vector<float>& observed)
{
    Eigen::Matrix<float, Eigen::Dynamic, 1> H(observed.size(), 1);
    for (size_t i = 0; i < observed.size(); i++)
        H(i) = observed.at(i);
    Eigen::Matrix<float, 1, Eigen::Dynamic> HT = H.transpose();
    
    Eigen::Matrix<float, Eigen::Dynamic, 1> z(simulated.size(), 1);
    for (size_t i = 0; i < simulated.size(); i++)
        z(i) = simulated.at(i);

    return (HT * H).inverse() * HT * z;
}

float LossFunction(const std::vector<float>& a, const std::vector<float>& b, float scale)
{
    float sum = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
        float error = a.at(i) - scale * b.at(i);
        sum += error * error; // squared error?
    }

    return sum;
}