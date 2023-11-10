#pragma once
#include <gsl_server/algorithms/Algorithm.h>
#include <deque>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.h>

namespace GSL
{
    class PlumeTracking : public Algorithm
    {
        friend class MovingStatePlumeTracking;

    public:
        PlumeTracking(rclcpp::Node::SharedPtr _node) : Algorithm(_node)
        {
        }

        virtual void initialize() override;

        void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) override;

    protected:
        double step_size;
        void declareParameters() override;

        void setExplorationGoal();
        geometry_msgs::msg::PoseStamped getRandomPoseInMap();
        void setSurgeGoal(double downWind_direction);
        virtual void setCastGoal(double downWind_direction) = 0;

        std::deque<float> lastConcentrationReadings;
        void gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg) override;
    };
} // namespace GSL