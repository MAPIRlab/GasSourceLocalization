#pragma once
#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

namespace GSL
{
    // map and costmap subscriptions live only while the algorithm is in this state. They are automatically deleted on exit.

    class WaitForMapState : public State
    {
    public:
        WaitForMapState(Algorithm* _algorithm) : State(_algorithm)
        {}

        void OnEnterState(State* previous) override;
        void OnExitState(State* previous) override;

        bool shouldWaitForGas = true;

    private:
        bool hasMap = false, hasCostmap = false;
        rclcpp::Subscription<OccupancyGrid>::SharedPtr mapSub;
        rclcpp::Subscription<OccupancyGrid>::SharedPtr costmapSub;

        void mapCallback(OccupancyGrid::SharedPtr msg);
        void costmapCallback(OccupancyGrid::SharedPtr msg);

        void setNextState();
    };
} // namespace GSL