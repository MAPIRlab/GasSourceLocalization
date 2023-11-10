#pragma once
#include <gsl_server/algorithms/Common/GSLState.h>
#include <rclcpp/rclcpp.hpp>
#include <gsl_server/core/ros_typedefs.h>

namespace GSL
{
    // map and costmap subscriptions live only while the algorithm is in this state. They are automatically deleted on exit.

    class WaitForMapState : public State
    {
    public:
        WaitForMapState(Algorithm* _algorithm) : State(_algorithm)
        {
        }

        void OnEnterState(State* previous) override;
        void OnExitState(State* previous) override;

    private:
        bool hasMap = false, hasCostmap = false;
        rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub;
        rclcpp::Subscription<OccupancyGrid>::SharedPtr costmap_sub;

        void mapCallback(OccupancyGrid::SharedPtr msg);
        void costmapCallback(OccupancyGrid::SharedPtr msg);
    };
} // namespace GSL