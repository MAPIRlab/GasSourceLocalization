#include "NACCompare.hpp"
#include <eigen3/Eigen/Dense>

namespace GSL
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

    float LeastSquaresScale(const std::vector<float>& simulated,
                            const std::vector<float>& observed,
                            const std::vector<float>& uncertainty)
    {
        Eigen::Matrix<float, Eigen::Dynamic, 1> H(observed.size(), 1);
        for (size_t i = 0; i < observed.size(); i++)
            H(i) = observed.at(i);
        Eigen::Matrix<float, 1, Eigen::Dynamic> HT = H.transpose();

        Eigen::Matrix<float, Eigen::Dynamic, 1> z(simulated.size(), 1);
        for (size_t i = 0; i < simulated.size(); i++)
            z(i) = simulated.at(i);

        Eigen::MatrixXf Q_inv = inverseCovariance(uncertainty);
        Eigen::Matrix<float, 1, 1> covarianceEstimation = (HT * Q_inv * H).inverse();
        return covarianceEstimation * HT * Q_inv * z;
    }

    float LossFunction(const std::vector<float>& simulated,
                       const std::vector<float>& observed,
                       const std::vector<float>& uncertainty,
                       float scale)
    {
        Eigen::Matrix<float, Eigen::Dynamic, 1> error(simulated.size(), 1);
        for (size_t i = 0; i < simulated.size(); i++)
            error(i) = simulated.at(i) - scale * observed.at(i);

        Eigen::MatrixXf Q_inv = inverseCovariance(uncertainty);

        return error.transpose() * Q_inv * error;
    }
} // namespace GSL