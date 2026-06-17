#include "NACCompare.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <gsl_server/core/Logging.hpp>

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

    float LeastSquaresScale(const std::vector<float>& simulated,
                            const std::vector<float>& observed,
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

    std::vector<float> LeastSquaresDoorwayCombination(const std::vector<std::vector<float>>& simulated,
                                                      const std::vector<float>& observed,
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
    template <typename T>
    T EvaluateScale(T scaledSimulated, T observed, T uncertainty)
    {
#if 1
        T diff = ceres::abs(ceres::log(observed + T(1)) - ceres::log(scaledSimulated + T(1)));
#else
        T diff = ceres::abs(observed - scaledSimulated);
#endif
        diff = ceres::lerp(diff, T(0), uncertainty);
        // T epsilon = T(1e-12); // pow is generally not differentiable at 0, which can cause nans to appear
        // T power(0.5);
        // diff = ceres::pow(diff + epsilon, T(power));
        return diff;
    }

    struct CostFunctorSingle
    {
        const float simulated;
        const float observed;
        const float uncertainty;

        template <typename T>
        bool operator()(const T* const x, T* residual) const
        {
            residual[0] = T(0);
            T scale = x[0];
            residual[0] += EvaluateScale(scale * T(simulated), T(observed), T(uncertainty));

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

    SingleScale FitSingleScale(const std::vector<float>& simulated,
                               const std::vector<float>& observed,
                               const std::vector<float>& uncertainty)
    {
        double x = 1.0;

        ceres::Problem problem;
        for (size_t i = 0; i < observed.size(); i++)
        {
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctorSingle, 1, 1>(
                new CostFunctorSingle{
                    .simulated = simulated.at(i),
                    .observed = observed.at(i),
                    .uncertainty = uncertainty.at(i)});
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), &x);
        }

        float residual = Solve(problem);
        return SingleScale{.scale = x, .residual = residual};
    }

    // Evaluates the solution (vector of scales) for a single cell
    struct CostFunctorDoorways
    {
        const std::vector<float>& simulated;
        const float& observed;
        const float& uncertainty;

        template <typename T>
        bool operator()(const T* const* x, T* residual) const
        {
            T sim = T(0);
            for (size_t doorwayIdx = 0; doorwayIdx < simulated.size(); doorwayIdx++)
            {
                T scale = *x[doorwayIdx];
                sim += scale * T(simulated.at(doorwayIdx));
            }

            residual[0] = EvaluateScale(sim, T(observed), T(uncertainty));

            GSL_ASSERT(ceres::isfinite(residual[0]));
            return true;
        }
    };

    MultipleScales FitDoorwayScales(const std::vector<std::vector<float>>& simulated,
                           const std::vector<float>& observed,
                           const std::vector<float>& uncertainty)
    {
        if (simulated.empty())
            return MultipleScales{.scales = {}, .residual = NAN};

        MultipleScales result;
        result.scales.resize(simulated.at(0).size(), 1.0);
        std::vector<double*> scale_pointers(result.scales.size());
        for (size_t i = 0; i < result.scales.size(); i++)
            scale_pointers.at(i) = &result.scales.at(i);

        ceres::Problem problem;
        // add one residual block for each cell in the map (at least, the ones with confidence > 0)
        for (size_t i = 0; i < observed.size(); i++)
        {
            auto* cost_function = new ceres::DynamicAutoDiffCostFunction<CostFunctorDoorways>(
                new CostFunctorDoorways{
                    .simulated = simulated.at(i),
                    .observed = observed.at(i),
                    .uncertainty = uncertainty.at(i)});

            for (size_t j = 0; j < result.scales.size(); j++)
                cost_function->AddParameterBlock(1);
            cost_function->SetNumResiduals(1);

            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), scale_pointers);
        }

        for (size_t i = 0; i < result.scales.size(); i++)
            problem.SetParameterLowerBound(scale_pointers.at(i), 0, 0.0);

        // Run the solver!
        result.residual = Solve(problem);
        return result;
    }
} // namespace GSL::NACCeres