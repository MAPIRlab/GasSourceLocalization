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

#include <ceres/ceres.h>

namespace GSL::NACCeres
{
    struct CostFunctorSingle
    {
        const std::vector<float>& simulated;
        const std::vector<float>& observed;
        const std::vector<float>& uncertainty;
        const float power = 0.5;

        template <typename T>
        bool operator()(const T* const x, T* residual) const
        {
            // TODO uncertainty
            residual[0] = T(0);
            for (size_t i = 0; i < observed.size(); i++)
            {
                T diff = T(observed.at(i)) - x[0] * T(simulated.at(i));
                diff = ceres::abs(diff);
                residual[0] += ceres::pow(diff, T(power));
            }

            return true;
        }
    };

    static float Solve(ceres::Problem& problem)
    {
        // Run the solver!
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        // options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // get the residual as a goodness of fit indicator
        ceres::Problem::EvaluateOptions evaluate_options;
        double cost;
        problem.Evaluate(evaluate_options, &cost, nullptr, nullptr, nullptr);
        return cost;
    }

    float FitSingleScale(const std::vector<float>& simulated,
                         const std::vector<float>& observed,
                         const std::vector<float>& uncertainty)
    {
        double x = 1.0;

        ceres::Problem problem;
        CostFunctorSingle cost_functor{.simulated = simulated, .observed = observed, .uncertainty = uncertainty};
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctorSingle, 1, 1>(&cost_functor);
        problem.AddResidualBlock(cost_function, nullptr, &x);

        return Solve(problem);
    }

    // Evaluates the solution (vector of scales) for a single cell
    struct CostFunctorDoorways
    {
        const std::vector<float>& simulated;
        const float& observed;
        const float& uncertainty;

        const float power = 0.5;

        template <typename T>
        bool operator()(const T* const* x, T* residual) const
        {
            // TODO uncertainty
            T diff = T(observed);
            for (size_t doorwayIdx = 0; doorwayIdx < simulated.size(); doorwayIdx++)
                diff -= ceres::abs(*x[doorwayIdx] * T(simulated.at(doorwayIdx)));

            residual[0] += ceres::pow(diff, T(power));

            return true;
        }
    };

    float FitDoorwayScales(const std::vector<float>& observed,
                           const std::vector<std::vector<float>>& simulated,
                           const std::vector<float>& uncertainty)
    {
        ceres::Problem problem;

        std::vector<double> scales(simulated.at(0).size(), 1.0);
        std::vector<double*> scale_pointers(scales.size());
        for (size_t i = 0; i < scales.size(); i++)
            scale_pointers.at(i) = &scales.at(i);

        // add one residual block for each cell in the map (at least, the ones with confidence > 0)
        for (size_t i = 0; i < observed.size(); i++)
        {
            auto* cost_function = new ceres::DynamicAutoDiffCostFunction<CostFunctorDoorways>(
                new CostFunctorDoorways{
                    .simulated = simulated.at(i),
                    .observed = observed.at(i),
                    .uncertainty = uncertainty.at(i)});

            for (size_t i = 0; i < scales.size(); i++)
                cost_function->AddParameterBlock(1);
            cost_function->SetNumResiduals(1);

            problem.AddResidualBlock(cost_function, nullptr, scale_pointers);
        }

        // Run the solver!
        return Solve(problem);
    }
} // namespace GSL::NACCeres