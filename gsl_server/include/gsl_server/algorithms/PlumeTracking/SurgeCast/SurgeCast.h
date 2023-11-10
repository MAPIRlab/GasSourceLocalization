#pragma once
#include <gsl_server/algorithms/PlumeTracking/PlumeTracking.h>

namespace GSL
{
    class SurgeCast : public PlumeTracking
    {
    public:
        SurgeCast(rclcpp::Node::SharedPtr _node) : PlumeTracking(_node)
        {
        }
        void setCastGoal(double downWind_direction) override;
    };
} // namespace GSL