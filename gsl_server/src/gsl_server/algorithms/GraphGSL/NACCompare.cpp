#include "NACCompare.hpp"
#include <eigen3/Eigen/Dense>

namespace GSL::NAC
{
    static Eigen::MatrixXf inverseCovariance(const std::vector<float>& uncertainty)
    {
#define ALLOW_COVARIANCES 0
#if ALLOW_COVARIANCES
        Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(uncertainty.size(), uncertainty.size());
        for (size_t i = 0; i < uncertainty.size(); i++)
            Q(i, i) = uncertainty.at(i);
        Eigen::MatrixXf Q_inv = Q.inverse();
#else
        Eigen::MatrixXf Q_inv = Eigen::MatrixXf::Zero(uncertainty.size(), uncertainty.size());
        for (size_t i = 0; i < uncertainty.size(); i++)
            Q_inv(i, i) = 1 / uncertainty.at(i);
#endif
        return Q_inv;
    }

    float LeastSquaresScale(const std::vector<float>& observed,
                            const std::vector<float>& simulated,
                            const std::vector<float>& uncertainty)
    {
        Eigen::Matrix<float, Eigen::Dynamic, 1> H(simulated.size(), 1);
        for (size_t i = 0; i < simulated.size(); i++)
            H(i) = simulated.at(i);
        Eigen::Matrix<float, 1, Eigen::Dynamic> HT = H.transpose();

        Eigen::Matrix<float, Eigen::Dynamic, 1> z(observed.size(), 1);
        for (size_t i = 0; i < observed.size(); i++)
            z(i) = observed.at(i);

        Eigen::MatrixXf Q_inv = inverseCovariance(uncertainty);
        Eigen::Matrix<float, 1, 1> covarianceEstimation = (HT * Q_inv * H).inverse();
        return covarianceEstimation * HT * Q_inv * z;
    }

    float Residual(const std::vector<float>& observed,
                   const std::vector<float>& simulated,
                   const std::vector<float>& confidence,
                   float scale)
    {
        float sum = 0;

        for (size_t i = 0; i < observed.size(); i++)
        {
            float diff = observed.at(i) - scale * simulated.at(i);
            diff = std::abs(diff);
            float error = std::pow(diff, 0.5);
            sum += std::lerp(0.0f, error, confidence.at(i));
        }

        return sum;
    }

    std::vector<float> LeastSquaresDoorwayCombination(const std::vector<float>& observed,
                                                      const std::vector<std::vector<float>>& simulated,
                                                      const std::vector<float>& uncertainty)
    {
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H(simulated.at(0).size(), simulated.size());
        for (size_t i = 0; i < simulated.size(); i++)
            for (size_t j = 0; j < simulated.at(0).size(); j++)
                H(j, i) = simulated.at(i).at(j);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> HT = H.transpose();

        Eigen::Matrix<float, Eigen::Dynamic, 1> z(observed.size(), 1);
        for (size_t i = 0; i < observed.size(); i++)
            z(i) = observed.at(i);

        Eigen::MatrixXf Q_inv = inverseCovariance(uncertainty);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> covarianceEstimation = (HT * Q_inv * H).inverse();
        Eigen::VectorXf result = covarianceEstimation * HT * Q_inv * z;
        return std::vector<float>(result.data(), result.data() + result.size());
    }

    float ResidualDoorways(const std::vector<float>& observed,
                           const std::vector<std::vector<float>>& simulated,
                           const std::vector<float>& confidence,
                           const std::vector<float>& weights)
    {
        float sum = 0;

        for (size_t i = 0; i < observed.size(); i++)
        {
            float simFinal = 0;
            for (size_t j = 0; j < simulated.size(); j++)
                simFinal += weights.at(j) * simulated.at(j).at(i);

            float diff = observed.at(i) - simFinal;
            float error = diff * diff;
            sum += std::lerp(0.0f, error, confidence.at(i));
        }

        return sum;
    }
} // namespace GSL::NAC