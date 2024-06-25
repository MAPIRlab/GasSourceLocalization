#pragma once
#include <gsl_server/algorithms/Algorithm.hpp>
#include <deque>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.hpp>

namespace GSL
{
    class PlumeTracking : public Algorithm
    {
        friend class MovingStatePlumeTracking;

    public:
        PlumeTracking(rclcpp::Node::SharedPtr _node) : Algorithm(_node)
        {
        }

        virtual void Initialize() override;

        virtual void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;

        virtual void OnUpdate() override;

    protected:
        double surgeStepSize;
        virtual void declareParameters() override;

        void setExplorationGoal();
        void setSurgeGoal(double upWind_direction);
        virtual void setCastGoal(double upWind_direction) = 0;

        std::deque<float> lastConcentrationReadings;
        float gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg) override;
    };
} // namespace GSL