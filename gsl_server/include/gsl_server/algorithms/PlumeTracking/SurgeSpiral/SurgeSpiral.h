#pragma once
#include <gsl_server/algorithms/PlumeTracking/PlumeTracking.h>

namespace GSL
{
    class SurgeSpiral : public PlumeTracking
    {
    public:
        SurgeSpiral(rclcpp::Node::SharedPtr _node) : PlumeTracking(_node)
        {
        }
        ~SurgeSpiral()
        {
        }

    protected:
        void setCastGoal(double downWind_direction) override;
        void declareParameters() override;
        void resetSpiral();
        NavigateToPose::Goal nextGoalSpiral(Pose initial);
        double spiralStepSize;
        double initSpiralStepSize;
        int spiral_iter;
        double spiralStep_increment;
    };
} // namespace GSL